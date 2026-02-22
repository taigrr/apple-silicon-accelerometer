//go:build darwin

package sensor

import (
	"encoding/binary"
	"fmt"
	"unsafe"

	"github.com/ebitengine/purego"
)

// Framework handles for IOKit and CoreFoundation.
var (
	iokit uintptr
	cf    uintptr
)

// IOKit functions.
var (
	ioServiceMatching              func(name *byte) uintptr
	ioServiceGetMatchingServices   func(mainPort uint32, matching uintptr, existing *uint32) int32
	ioIteratorNext                 func(iterator uint32) uint32
	ioObjectRelease                func(object uint32) int32
	ioRegistryEntryCreateCFProp    func(entry uint32, key uintptr, allocator uintptr, options uint32) uintptr
	ioRegistryEntrySetCFProp       func(entry uint32, key uintptr, value uintptr) int32
	ioHIDDeviceCreate              func(allocator uintptr, service uint32) uintptr
	ioHIDDeviceOpen                func(device uintptr, options int32) int32
	ioHIDDeviceRegisterInputReport func(device uintptr, report uintptr, reportLen int, callback uintptr, context uintptr)
	ioHIDDeviceScheduleWithRL      func(device uintptr, runLoop uintptr, mode uintptr)
)

// CoreFoundation functions.
var (
	cfStringCreateWithCString func(alloc uintptr, cStr *byte, encoding uint32) uintptr
	cfNumberCreate            func(alloc uintptr, theType int32, valuePtr uintptr) uintptr
	cfNumberGetValue          func(number uintptr, theType int32, valuePtr uintptr) bool
	cfRunLoopGetCurrent       func() uintptr
	cfRunLoopRunInMode        func(mode uintptr, seconds float64, returnAfterSourceHandled bool) int32
)

// CoreFoundation global constants.
var (
	kCFAllocatorDefault   uintptr
	kCFRunLoopDefaultMode uintptr
)

func init() {
	var err error
	iokit, err = purego.Dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", purego.RTLD_LAZY)
	if err != nil {
		panic(fmt.Sprintf("dlopen IOKit: %v", err))
	}

	cf, err = purego.Dlopen("/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation", purego.RTLD_LAZY)
	if err != nil {
		panic(fmt.Sprintf("dlopen CoreFoundation: %v", err))
	}

	// IOKit functions
	purego.RegisterLibFunc(&ioServiceMatching, iokit, "IOServiceMatching")
	purego.RegisterLibFunc(&ioServiceGetMatchingServices, iokit, "IOServiceGetMatchingServices")
	purego.RegisterLibFunc(&ioIteratorNext, iokit, "IOIteratorNext")
	purego.RegisterLibFunc(&ioObjectRelease, iokit, "IOObjectRelease")
	purego.RegisterLibFunc(&ioRegistryEntryCreateCFProp, iokit, "IORegistryEntryCreateCFProperty")
	purego.RegisterLibFunc(&ioRegistryEntrySetCFProp, iokit, "IORegistryEntrySetCFProperty")
	purego.RegisterLibFunc(&ioHIDDeviceCreate, iokit, "IOHIDDeviceCreate")
	purego.RegisterLibFunc(&ioHIDDeviceOpen, iokit, "IOHIDDeviceOpen")
	purego.RegisterLibFunc(&ioHIDDeviceRegisterInputReport, iokit, "IOHIDDeviceRegisterInputReportCallback")
	purego.RegisterLibFunc(&ioHIDDeviceScheduleWithRL, iokit, "IOHIDDeviceScheduleWithRunLoop")

	// CoreFoundation functions
	purego.RegisterLibFunc(&cfStringCreateWithCString, cf, "CFStringCreateWithCString")
	purego.RegisterLibFunc(&cfNumberCreate, cf, "CFNumberCreate")
	purego.RegisterLibFunc(&cfNumberGetValue, cf, "CFNumberGetValue")
	purego.RegisterLibFunc(&cfRunLoopGetCurrent, cf, "CFRunLoopGetCurrent")
	purego.RegisterLibFunc(&cfRunLoopRunInMode, cf, "CFRunLoopRunInMode")

	// Global constants — these are pointers-to-pointers in the dylib,
	// so we dereference once to get the actual CFTypeRef value.
	kCFAllocatorDefault = derefSymbol(cf, "kCFAllocatorDefault")
	kCFRunLoopDefaultMode = derefSymbol(cf, "kCFRunLoopDefaultMode")
}

// derefSymbol loads a symbol from a dylib and dereferences it (pointer-to-pointer).
//
//go:nosplit
func derefSymbol(lib uintptr, name string) uintptr {
	sym, _ := purego.Dlsym(lib, name)
	if sym == 0 {
		return 0
	}
	// The symbol is a global variable holding a pointer.
	// We need to read the pointer value at that address.
	return **(**uintptr)(unsafe.Pointer(&sym))
}

// cfStr creates a CFString from a Go string.
func cfStr(s string) uintptr {
	return cfStringCreateWithCString(0, cStr(s), CFStringEncodingUTF8)
}

// cfNum32 creates a CFNumber (SInt32) from an int32.
func cfNum32(v int32) uintptr {
	return cfNumberCreate(0, CFNumberSInt32Type, uintptr(unsafe.Pointer(&v)))
}

// cStr converts a Go string to a null-terminated C string.
func cStr(s string) *byte {
	b := make([]byte, len(s)+1)
	copy(b, s)
	return &b[0]
}

// propInt reads an integer IORegistry property from a service.
func propInt(service uint32, key string) (int64, bool) {
	ref := ioRegistryEntryCreateCFProp(service, cfStr(key), 0, 0)
	if ref == 0 {
		return 0, false
	}
	var val int64
	if !cfNumberGetValue(ref, CFNumberSInt64Type, uintptr(unsafe.Pointer(&val))) {
		return 0, false
	}
	return val, true
}

// ReportCallback is the type for HID input report callbacks.
// Parameters: context, result, sender, reportType, reportID, report data, report length.
type ReportCallback func(context uintptr, result int32, sender uintptr, reportType int32, reportID uint32, report []byte)

// reportCallbackTrampoline is a raw callback that extracts report data and
// dispatches to registered Go callbacks.
var activeCallbacks = make(map[uintptr]ReportCallback)
var nextCallbackID uintptr

// ParseIMUReport extracts 3 int32 XYZ values from a BMI286 IMU report.
func ParseIMUReport(data []byte) (x, y, z int32) {
	if len(data) < IMUDataOffset+12 {
		return 0, 0, 0
	}
	off := IMUDataOffset
	x = int32(binary.LittleEndian.Uint32(data[off:]))
	y = int32(binary.LittleEndian.Uint32(data[off+4:]))
	z = int32(binary.LittleEndian.Uint32(data[off+8:]))
	return x, y, z
}

// ParseLidAngle extracts the lid angle from a lid sensor report.
func ParseLidAngle(data []byte) (float32, bool) {
	if len(data) < LidReportLen {
		return 0, false
	}
	if data[0] != 1 {
		return 0, false
	}
	angle := binary.LittleEndian.Uint16(data[1:3]) & 0x1FF
	return float32(angle), true
}
