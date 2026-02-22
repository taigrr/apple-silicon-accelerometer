//go:build darwin

// Package shm provides POSIX shared memory ring buffers and snapshot regions
// compatible with the Python spu_sensor.py layout.
package shm

import (
	"encoding/binary"
	"fmt"
	"math"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Ring buffer and snapshot constants matching the Python layout.
const (
	RingCap    = 8000
	RingEntry  = 12 // 3x int32: x, y, z
	SHMHeader  = 16 // [0..3] write_idx u32, [4..11] total u64, [12..15] restarts u32
	SHMSize    = SHMHeader + RingCap*RingEntry
	SnapHeader = 8 // [0..3] update_count u32, [4..7] pad

	AccelScale = 65536.0 // Q16 raw -> g
	GyroScale  = 65536.0 // Q16 raw -> deg/s

	NameAccel = "vib_detect_shm"
	NameGyro  = "vib_detect_shm_gyro"
	NameALS   = "vib_detect_shm_als"
	NameLid   = "vib_detect_shm_lid"

	ALSReportLen = 122
	LidReportLen = 3

	ALSSize = SnapHeader + ALSReportLen
	LidSize = SnapHeader + 4
)

// Sample holds a 3-axis reading scaled to real units (g or deg/s).
type Sample struct {
	X, Y, Z float64
}

// RingBuffer is a shared memory ring buffer for high-rate IMU data.
type RingBuffer struct {
	buf  []byte
	name string
	fd   int
}

// CreateRing creates a new POSIX shared memory ring buffer.
func CreateRing(name string) (*RingBuffer, error) {
	// Unlink any stale segment first.
	_ = shmUnlink(name)

	fd, err := shmOpen(name, unix.O_CREAT|unix.O_RDWR, 0600)
	if err != nil {
		return nil, fmt.Errorf("shm_open %s: %w", name, err)
	}

	if err := unix.Ftruncate(fd, SHMSize); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("ftruncate %s: %w", name, err)
	}

	buf, err := unix.Mmap(fd, 0, SHMSize, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("mmap %s: %w", name, err)
	}

	// Zero the buffer.
	clear(buf)

	return &RingBuffer{buf: buf, name: name, fd: fd}, nil
}

// OpenRing opens an existing POSIX shared memory ring buffer (read-only).
func OpenRing(name string) (*RingBuffer, error) {
	fd, err := shmOpen(name, unix.O_RDONLY, 0)
	if err != nil {
		return nil, fmt.Errorf("shm_open %s: %w", name, err)
	}

	buf, err := unix.Mmap(fd, 0, SHMSize, unix.PROT_READ, unix.MAP_SHARED)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("mmap %s: %w", name, err)
	}

	return &RingBuffer{buf: buf, name: name, fd: fd}, nil
}

// WriteSample writes a raw int32 XYZ sample into the ring buffer.
func (r *RingBuffer) WriteSample(x, y, z int32) {
	idx := binary.LittleEndian.Uint32(r.buf[0:4])
	off := SHMHeader + int(idx)*RingEntry

	binary.LittleEndian.PutUint32(r.buf[off:], uint32(x))
	binary.LittleEndian.PutUint32(r.buf[off+4:], uint32(y))
	binary.LittleEndian.PutUint32(r.buf[off+8:], uint32(z))

	binary.LittleEndian.PutUint32(r.buf[0:4], (idx+1)%RingCap)
	total := binary.LittleEndian.Uint64(r.buf[4:12])
	binary.LittleEndian.PutUint64(r.buf[4:12], total+1)
}

// SetRestarts writes the restart counter in the header.
func (r *RingBuffer) SetRestarts(count uint32) {
	binary.LittleEndian.PutUint32(r.buf[12:16], count)
}

// ReadNew reads new samples since lastTotal, scaling by the given factor.
// Returns the samples and the new total.
func (r *RingBuffer) ReadNew(lastTotal uint64, scale float64) ([]Sample, uint64) {
	total := binary.LittleEndian.Uint64(r.buf[4:12])
	nNew := int64(total) - int64(lastTotal)
	if nNew <= 0 {
		return nil, total
	}
	if nNew > RingCap {
		nNew = RingCap
	}

	idx := binary.LittleEndian.Uint32(r.buf[0:4])
	start := (int64(idx) - nNew + RingCap) % RingCap
	samples := make([]Sample, nNew)

	for i := int64(0); i < nNew; i++ {
		pos := (start + i) % RingCap
		off := SHMHeader + int(pos)*RingEntry
		x := int32(binary.LittleEndian.Uint32(r.buf[off:]))
		y := int32(binary.LittleEndian.Uint32(r.buf[off+4:]))
		z := int32(binary.LittleEndian.Uint32(r.buf[off+8:]))
		samples[i] = Sample{
			X: float64(x) / scale,
			Y: float64(y) / scale,
			Z: float64(z) / scale,
		}
	}

	return samples, total
}

// Close unmaps and closes the shared memory (does not unlink).
func (r *RingBuffer) Close() error {
	if err := unix.Munmap(r.buf); err != nil {
		return err
	}
	return unix.Close(r.fd)
}

// Unlink removes the named shared memory segment.
func (r *RingBuffer) Unlink() error {
	return shmUnlink(r.name)
}

// Snapshot is a shared memory region for low-rate sensors (latest value only).
type Snapshot struct {
	buf  []byte
	name string
	fd   int
	size int
}

// CreateSnapshot creates a new POSIX shared memory snapshot region.
func CreateSnapshot(name string, size int) (*Snapshot, error) {
	_ = shmUnlink(name)

	fd, err := shmOpen(name, unix.O_CREAT|unix.O_RDWR, 0600)
	if err != nil {
		return nil, fmt.Errorf("shm_open %s: %w", name, err)
	}

	if err := unix.Ftruncate(fd, int64(size)); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("ftruncate %s: %w", name, err)
	}

	buf, err := unix.Mmap(fd, 0, size, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("mmap %s: %w", name, err)
	}

	clear(buf)

	return &Snapshot{buf: buf, name: name, fd: fd, size: size}, nil
}

// OpenSnapshot opens an existing shared memory snapshot region (read-only).
func OpenSnapshot(name string, size int) (*Snapshot, error) {
	fd, err := shmOpen(name, unix.O_RDONLY, 0)
	if err != nil {
		return nil, fmt.Errorf("shm_open %s: %w", name, err)
	}

	buf, err := unix.Mmap(fd, 0, size, unix.PROT_READ, unix.MAP_SHARED)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("mmap %s: %w", name, err)
	}

	return &Snapshot{buf: buf, name: name, fd: fd, size: size}, nil
}

// Write writes a payload into the snapshot region and increments the counter.
func (s *Snapshot) Write(payload []byte) {
	copy(s.buf[SnapHeader:], payload)
	cnt := binary.LittleEndian.Uint32(s.buf[0:4])
	binary.LittleEndian.PutUint32(s.buf[0:4], cnt+1)
}

// WriteFloat32 writes a single float32 value to the snapshot payload.
func (s *Snapshot) WriteFloat32(v float32) {
	binary.LittleEndian.PutUint32(s.buf[SnapHeader:], math.Float32bits(v))
	cnt := binary.LittleEndian.Uint32(s.buf[0:4])
	binary.LittleEndian.PutUint32(s.buf[0:4], cnt+1)
}

// Read reads the snapshot payload if the counter has changed since lastCount.
// Returns nil payload if unchanged.
func (s *Snapshot) Read(lastCount uint32, payloadLen int) ([]byte, uint32) {
	cnt := binary.LittleEndian.Uint32(s.buf[0:4])
	if cnt == lastCount {
		return nil, cnt
	}
	payload := make([]byte, payloadLen)
	copy(payload, s.buf[SnapHeader:SnapHeader+payloadLen])
	return payload, cnt
}

// ReadFloat32 reads a float32 value from the snapshot if the counter changed.
func (s *Snapshot) ReadFloat32(lastCount uint32) (float32, uint32, bool) {
	cnt := binary.LittleEndian.Uint32(s.buf[0:4])
	if cnt == lastCount {
		return 0, cnt, false
	}
	bits := binary.LittleEndian.Uint32(s.buf[SnapHeader:])
	return math.Float32frombits(bits), cnt, true
}

// Close unmaps and closes the snapshot shared memory.
func (s *Snapshot) Close() error {
	if err := unix.Munmap(s.buf); err != nil {
		return err
	}
	return unix.Close(s.fd)
}

// Unlink removes the named shared memory segment.
func (s *Snapshot) Unlink() error {
	return shmUnlink(s.name)
}

// shmOpen wraps the shm_open syscall via /dev/shm path on macOS.
func shmOpen(name string, flags int, mode uint32) (int, error) {
	// On macOS, shm_open is a libc function. We use the syscall package
	// approach of opening /dev/shm/<name> which doesn't exist on macOS.
	// Instead, use cgo-free purego to call shm_open.
	return shmOpenDarwin(name, flags, mode)
}

func shmUnlink(name string) error {
	return shmUnlinkDarwin(name)
}

// shmOpenDarwin and shmUnlinkDarwin use purego to call libc shm_open/shm_unlink.
// These are defined in shm_cgo_darwin.go.

// Ptr converts an unsafe.Pointer-sized value. Helper for purego calls.
func ptrToUintptr(p unsafe.Pointer) uintptr {
	return uintptr(p)
}
