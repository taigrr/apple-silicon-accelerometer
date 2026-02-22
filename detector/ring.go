package detector

// RingFloat is a fixed-capacity ring buffer for float64 values.
type RingFloat struct {
	data []float64
	pos  int
	full bool
	cap  int
}

// NewRingFloat creates a RingFloat with the given capacity.
func NewRingFloat(cap int) *RingFloat {
	return &RingFloat{
		data: make([]float64, cap),
		cap:  cap,
	}
}

// Push adds a value to the ring buffer.
func (r *RingFloat) Push(v float64) {
	r.data[r.pos] = v
	r.pos++
	if r.pos >= r.cap {
		r.pos = 0
		r.full = true
	}
}

// Len returns the number of elements in the buffer.
func (r *RingFloat) Len() int {
	if r.full {
		return r.cap
	}
	return r.pos
}

// Slice returns the buffer contents in insertion order.
func (r *RingFloat) Slice() []float64 {
	n := r.Len()
	out := make([]float64, n)
	if r.full {
		copy(out, r.data[r.pos:])
		copy(out[r.cap-r.pos:], r.data[:r.pos])
	} else {
		copy(out, r.data[:r.pos])
	}
	return out
}

// Vec3 holds a 3D vector.
type Vec3 struct {
	X, Y, Z float64
}

// RingVec3 is a fixed-capacity ring buffer for Vec3 values.
type RingVec3 struct {
	data []Vec3
	pos  int
	full bool
	cap  int
}

// NewRingVec3 creates a RingVec3 with the given capacity.
func NewRingVec3(cap int) *RingVec3 {
	return &RingVec3{
		data: make([]Vec3, cap),
		cap:  cap,
	}
}

// Push3 adds XYZ values to the ring.
func (r *RingVec3) Push3(x, y, z float64) {
	r.data[r.pos] = Vec3{x, y, z}
	r.pos++
	if r.pos >= r.cap {
		r.pos = 0
		r.full = true
	}
}

// Len returns the number of elements.
func (r *RingVec3) Len() int {
	if r.full {
		return r.cap
	}
	return r.pos
}

// Slice returns the buffer contents in insertion order.
func (r *RingVec3) Slice() []Vec3 {
	n := r.Len()
	out := make([]Vec3, n)
	if r.full {
		copy(out, r.data[r.pos:])
		copy(out[r.cap-r.pos:], r.data[:r.pos])
	} else {
		copy(out, r.data[:r.pos])
	}
	return out
}
