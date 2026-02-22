//go:build darwin

// Package keyboard controls Apple Silicon MacBook keyboard backlight brightness
// via the private CoreBrightness.framework (KeyboardBrightnessClient).
package keyboard

import (
	"fmt"
	"unsafe"

	"github.com/ebitengine/purego"
)

var (
	objcRuntime uintptr

	// Objective-C runtime functions
	objcGetClass   func(name *byte) uintptr
	objcSelRegName func(name *byte) uintptr
	objcMsgSend    func(receiver uintptr, sel uintptr, args ...uintptr) uintptr

	// For float returns (objc_msgSend_fpret on x86_64, objc_msgSend on arm64)
	objcMsgSendFpret func(receiver uintptr, sel uintptr, args ...uintptr) float32

	// Selectors
	selAlloc                uintptr
	selInit                 uintptr
	selSetBrightnessFade    uintptr
	selSetBrightness        uintptr
	selBrightnessForKB      uintptr
	selEnableAutoBrightness uintptr
	selSetIdleDimTime       uintptr
	selSuspendIdleDimming   uintptr
	selLoad                 uintptr

	initialized bool
)

func initRuntime() {
	if initialized {
		return
	}

	var err error
	objcRuntime, err = purego.Dlopen("/usr/lib/libobjc.A.dylib", purego.RTLD_LAZY)
	if err != nil {
		panic(fmt.Sprintf("dlopen libobjc: %v", err))
	}

	purego.RegisterLibFunc(&objcGetClass, objcRuntime, "objc_getClass")
	purego.RegisterLibFunc(&objcSelRegName, objcRuntime, "sel_registerName")
	purego.RegisterLibFunc(&objcMsgSend, objcRuntime, "objc_msgSend")
	purego.RegisterLibFunc(&objcMsgSendFpret, objcRuntime, "objc_msgSend")

	selAlloc = sel("alloc")
	selInit = sel("init")
	selSetBrightnessFade = sel("setBrightness:fadeSpeed:commit:forKeyboard:")
	selSetBrightness = sel("setBrightness:forKeyboard:")
	selBrightnessForKB = sel("brightnessForKeyboard:")
	selEnableAutoBrightness = sel("enableAutoBrightness:forKeyboard:")
	selSetIdleDimTime = sel("setIdleDimTime:forKeyboard:")
	selSuspendIdleDimming = sel("suspendIdleDimming:forKeyboard:")
	selLoad = sel("load")

	initialized = true
}

func sel(name string) uintptr {
	b := make([]byte, len(name)+1)
	copy(b, name)
	return objcSelRegName(&b[0])
}

func cls(name string) uintptr {
	b := make([]byte, len(name)+1)
	copy(b, name)
	return objcGetClass(&b[0])
}

// Client controls the keyboard backlight via CoreBrightness.
type Client struct {
	instance  uintptr
	keyboardID uintptr
}

// NewClient loads CoreBrightness.framework and creates a KeyboardBrightnessClient.
// The keyboardID is typically 1 for the built-in keyboard.
func NewClient(keyboardID uint64) (*Client, error) {
	initRuntime()

	// Load CoreBrightness.framework
	nsBundleCls := cls("NSBundle")
	if nsBundleCls == 0 {
		return nil, fmt.Errorf("NSBundle class not found")
	}

	path := newNSString("/System/Library/PrivateFrameworks/CoreBrightness.framework")
	bundleWithPath := sel("bundleWithPath:")
	bundle := objcMsgSend(nsBundleCls, bundleWithPath, path)
	if bundle == 0 {
		return nil, fmt.Errorf("failed to load CoreBrightness.framework bundle")
	}

	loaded := objcMsgSend(bundle, selLoad)
	_ = loaded

	kbcClass := cls("KeyboardBrightnessClient")
	if kbcClass == 0 {
		return nil, fmt.Errorf("KeyboardBrightnessClient class not found (CoreBrightness not loaded?)")
	}

	instance := objcMsgSend(kbcClass, selAlloc)
	instance = objcMsgSend(instance, selInit)
	if instance == 0 {
		return nil, fmt.Errorf("failed to create KeyboardBrightnessClient instance")
	}

	return &Client{
		instance:   instance,
		keyboardID: uintptr(keyboardID),
	}, nil
}

// SetBrightness sets the keyboard backlight brightness (0.0–1.0) with a fade
// duration in milliseconds.
func (c *Client) SetBrightness(level float32, fadeMs int) {
	lvl := uintptr(*(*uint32)(unsafe.Pointer(&level)))
	fade := uintptr(fadeMs)
	commit := uintptr(1) // YES
	objcMsgSend(c.instance, selSetBrightnessFade, lvl, fade, commit, c.keyboardID)
}

// GetBrightness returns the current keyboard backlight brightness (0.0–1.0).
func (c *Client) GetBrightness() float32 {
	return objcMsgSendFpret(c.instance, selBrightnessForKB, c.keyboardID)
}

// SetAutoBrightness enables or disables auto-brightness for the keyboard.
func (c *Client) SetAutoBrightness(enabled bool) {
	var val uintptr
	if enabled {
		val = 1
	}
	objcMsgSend(c.instance, selEnableAutoBrightness, val, c.keyboardID)
}

// SetIdleDimTime sets the idle dim timeout in seconds. Set to 0 to disable.
func (c *Client) SetIdleDimTime(seconds float64) {
	bits := *(*uintptr)(unsafe.Pointer(&seconds))
	objcMsgSend(c.instance, selSetIdleDimTime, bits, c.keyboardID)
}

// SuspendIdleDimming suspends or resumes idle dimming.
func (c *Client) SuspendIdleDimming(suspend bool) {
	var val uintptr
	if suspend {
		val = 1
	}
	objcMsgSend(c.instance, selSuspendIdleDimming, val, c.keyboardID)
}

// newNSString creates an NSString from a Go string via the Objective-C runtime.
func newNSString(s string) uintptr {
	nsCls := cls("NSString")
	selStr := sel("stringWithUTF8String:")
	b := make([]byte, len(s)+1)
	copy(b, s)
	return objcMsgSend(nsCls, selStr, uintptr(unsafe.Pointer(&b[0])))
}
