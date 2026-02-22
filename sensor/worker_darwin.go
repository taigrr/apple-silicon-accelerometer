//go:build darwin

package sensor

import (
	"fmt"
	"runtime"
	"unsafe"

	"github.com/ebitengine/purego"
	"github.com/taigrr/apple-silicon-accelerometer/shm"
)

// Config holds the shared memory targets for the sensor worker.
type Config struct {
	AccelRing *shm.RingBuffer
	GyroRing  *shm.RingBuffer
	ALSSnap   *shm.Snapshot
	LidSnap   *shm.Snapshot
	Restarts  uint32
}

// callbackState holds mutable state accessed from C callbacks.
// Must be kept alive (prevent GC) for the duration of the run loop.
type callbackState struct {
	accelRing *shm.RingBuffer
	gyroRing  *shm.RingBuffer
	alsSnap   *shm.Snapshot
	lidSnap   *shm.Snapshot
	accelDec  int
	gyroDec   int
}

// Global state pointer — only one worker per process.
var globalState *callbackState

// Raw C callback function pointers registered via purego.
var (
	accelCallbackPtr uintptr
	gyroCallbackPtr  uintptr
	alsCallbackPtr   uintptr
	lidCallbackPtr   uintptr
)

func init() {
	// Register callback trampolines.
	accelCallbackPtr = purego.NewCallback(accelCallback)
	gyroCallbackPtr = purego.NewCallback(gyroCallback)
	alsCallbackPtr = purego.NewCallback(alsCallback)
	lidCallbackPtr = purego.NewCallback(lidCallback)
}

// accelCallback is the raw HID callback for accelerometer reports.
func accelCallback(_ uintptr, _ int32, _ uintptr, _ int32, _ uint32, report *byte, length int) {
	if globalState == nil || globalState.accelRing == nil || length != IMUReportLen {
		return
	}
	globalState.accelDec++
	if globalState.accelDec < IMUDecimation {
		return
	}
	globalState.accelDec = 0

	data := unsafe.Slice(report, length)
	x, y, z := ParseIMUReport(data)
	globalState.accelRing.WriteSample(x, y, z)
}

// gyroCallback is the raw HID callback for gyroscope reports.
func gyroCallback(_ uintptr, _ int32, _ uintptr, _ int32, _ uint32, report *byte, length int) {
	if globalState == nil || globalState.gyroRing == nil || length != IMUReportLen {
		return
	}
	globalState.gyroDec++
	if globalState.gyroDec < IMUDecimation {
		return
	}
	globalState.gyroDec = 0

	data := unsafe.Slice(report, length)
	x, y, z := ParseIMUReport(data)
	globalState.gyroRing.WriteSample(x, y, z)
}

// alsCallback is the raw HID callback for ambient light sensor reports.
func alsCallback(_ uintptr, _ int32, _ uintptr, _ int32, _ uint32, report *byte, length int) {
	if globalState == nil || globalState.alsSnap == nil || length != ALSReportLen {
		return
	}
	data := unsafe.Slice(report, length)
	payload := make([]byte, ALSReportLen)
	copy(payload, data)
	globalState.alsSnap.Write(payload)
}

// lidCallback is the raw HID callback for lid angle reports.
func lidCallback(_ uintptr, _ int32, _ uintptr, _ int32, _ uint32, report *byte, length int) {
	if globalState == nil || globalState.lidSnap == nil || length < LidReportLen {
		return
	}
	data := unsafe.Slice(report, length)
	angle, ok := ParseLidAngle(data)
	if !ok {
		return
	}
	globalState.lidSnap.WriteFloat32(angle)
}

// Run starts the sensor worker. This function blocks forever, running the
// CFRunLoop to receive HID callbacks. It must be called from the main thread.
func Run(cfg Config) error {
	// Lock this goroutine to the OS thread (required for CFRunLoop).
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()

	globalState = &callbackState{
		accelRing: cfg.AccelRing,
		gyroRing:  cfg.GyroRing,
		alsSnap:   cfg.ALSSnap,
		lidSnap:   cfg.LidSnap,
	}

	if cfg.AccelRing != nil {
		cfg.AccelRing.SetRestarts(cfg.Restarts)
	}

	// Wake up SPU drivers by setting properties.
	if err := wakeSPUDrivers(); err != nil {
		return fmt.Errorf("waking SPU drivers: %w", err)
	}

	// Enumerate HID devices and register callbacks.
	if err := registerHIDDevices(); err != nil {
		return fmt.Errorf("registering HID devices: %w", err)
	}

	// Run the CFRunLoop forever (handles HID callbacks).
	for {
		cfRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false)
	}
}

// wakeSPUDrivers finds AppleSPUHIDDriver services and sets sensor properties.
func wakeSPUDrivers() error {
	matching := ioServiceMatching(cStr("AppleSPUHIDDriver"))
	var it uint32
	kr := ioServiceGetMatchingServices(0, matching, &it)
	if kr != 0 {
		return fmt.Errorf("IOServiceGetMatchingServices returned %d", kr)
	}

	for {
		svc := ioIteratorNext(it)
		if svc == 0 {
			break
		}

		props := []struct {
			key string
			val int32
		}{
			{"SensorPropertyReportingState", 1},
			{"SensorPropertyPowerState", 1},
			{"ReportInterval", ReportIntervalUS},
		}
		for _, p := range props {
			ioRegistryEntrySetCFProp(svc, cfStr(p.key), cfNum32(p.val))
		}
		ioObjectRelease(svc)
	}

	return nil
}

// gcRoots holds references that must not be garbage collected during the run loop.
var gcRoots []any

// registerHIDDevices enumerates AppleSPUHIDDevice services and registers
// the appropriate callback for each sensor type.
func registerHIDDevices() error {
	matching := ioServiceMatching(cStr("AppleSPUHIDDevice"))
	var it uint32
	kr := ioServiceGetMatchingServices(0, matching, &it)
	if kr != 0 {
		return fmt.Errorf("IOServiceGetMatchingServices returned %d", kr)
	}

	for {
		svc := ioIteratorNext(it)
		if svc == 0 {
			break
		}

		up, _ := propInt(svc, "PrimaryUsagePage")
		u, _ := propInt(svc, "PrimaryUsage")

		var cb uintptr
		switch {
		case up == PageVendor && u == UsageAccel:
			cb = accelCallbackPtr
		case up == PageVendor && u == UsageGyro && globalState.gyroRing != nil:
			cb = gyroCallbackPtr
		case up == PageVendor && u == UsageALS && globalState.alsSnap != nil:
			cb = alsCallbackPtr
		case up == PageSensor && u == UsageLid && globalState.lidSnap != nil:
			cb = lidCallbackPtr
		}

		if cb != 0 {
			hid := ioHIDDeviceCreate(kCFAllocatorDefault, svc)
			if hid != 0 {
				kr := ioHIDDeviceOpen(hid, 0)
				if kr == 0 {
					reportBuf := make([]byte, ReportBufSize)
					gcRoots = append(gcRoots, reportBuf)
					bufPtr := uintptr(unsafe.Pointer(&reportBuf[0]))

					ioHIDDeviceRegisterInputReport(hid, bufPtr, ReportBufSize, cb, 0)
					ioHIDDeviceScheduleWithRL(hid, cfRunLoopGetCurrent(), kCFRunLoopDefaultMode)
				}
			}
		}
		ioObjectRelease(svc)
	}

	return nil
}
