//go:build darwin

package shm

import (
	"fmt"
	"unsafe"

	"github.com/ebitengine/purego"
)

var (
	libSystem     uintptr
	fnShmOpen     func(name *byte, oflag int32, mode uint16) int32
	fnShmUnlink   func(name *byte) int32
	syscallsReady bool
)

func initSyscalls() {
	if syscallsReady {
		return
	}
	var err error
	libSystem, err = purego.Dlopen("/usr/lib/libSystem.B.dylib", purego.RTLD_LAZY)
	if err != nil {
		panic(fmt.Sprintf("dlopen libSystem: %v", err))
	}

	purego.RegisterLibFunc(&fnShmOpen, libSystem, "shm_open")
	purego.RegisterLibFunc(&fnShmUnlink, libSystem, "shm_unlink")
	syscallsReady = true
}

func cString(s string) *byte {
	b := make([]byte, len(s)+1)
	copy(b, s)
	b[len(s)] = 0
	return &b[0]
}

func shmOpenDarwin(name string, flags int, mode uint32) (int, error) {
	initSyscalls()
	// shm_open names must start with /
	shmName := "/" + name
	fd := fnShmOpen(cString(shmName), int32(flags), uint16(mode))
	if fd < 0 {
		return -1, fmt.Errorf("shm_open(%q) returned %d", shmName, fd)
	}
	return int(fd), nil
}

func shmUnlinkDarwin(name string) error {
	initSyscalls()
	shmName := "/" + name
	ret := fnShmUnlink(cString(shmName))
	if ret < 0 {
		return fmt.Errorf("shm_unlink(%q) returned %d", shmName, ret)
	}
	return nil
}

// Ensure unsafe is used (for purego compatibility).
var _ = unsafe.Pointer(nil)
