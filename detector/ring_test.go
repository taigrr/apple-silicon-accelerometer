package detector

import (
	"testing"
)

func TestRingFloat_PushAndLen(t *testing.T) {
	r := NewRingFloat(5)
	if r.Len() != 0 {
		t.Fatalf("expected len 0, got %d", r.Len())
	}

	r.Push(1.0)
	r.Push(2.0)
	r.Push(3.0)
	if r.Len() != 3 {
		t.Fatalf("expected len 3, got %d", r.Len())
	}

	// Fill to capacity
	r.Push(4.0)
	r.Push(5.0)
	if r.Len() != 5 {
		t.Fatalf("expected len 5, got %d", r.Len())
	}

	// Overflow wraps
	r.Push(6.0)
	if r.Len() != 5 {
		t.Fatalf("expected len 5 after overflow, got %d", r.Len())
	}
}

func TestRingFloat_SliceOrder(t *testing.T) {
	r := NewRingFloat(4)
	r.Push(10)
	r.Push(20)
	r.Push(30)

	got := r.Slice()
	want := []float64{10, 20, 30}
	if len(got) != len(want) {
		t.Fatalf("len mismatch: got %d, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("index %d: got %f, want %f", i, got[i], want[i])
		}
	}
}

func TestRingFloat_SliceAfterWrap(t *testing.T) {
	r := NewRingFloat(3)
	r.Push(1)
	r.Push(2)
	r.Push(3)
	r.Push(4) // overwrites 1
	r.Push(5) // overwrites 2

	got := r.Slice()
	want := []float64{3, 4, 5}
	if len(got) != len(want) {
		t.Fatalf("len mismatch: got %d, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("index %d: got %f, want %f", i, got[i], want[i])
		}
	}
}

func TestRingFloat_EmptySlice(t *testing.T) {
	r := NewRingFloat(10)
	got := r.Slice()
	if len(got) != 0 {
		t.Fatalf("expected empty slice, got len %d", len(got))
	}
}

func TestRingVec3_PushAndLen(t *testing.T) {
	r := NewRingVec3(3)
	if r.Len() != 0 {
		t.Fatalf("expected len 0, got %d", r.Len())
	}

	r.Push3(1, 2, 3)
	r.Push3(4, 5, 6)
	if r.Len() != 2 {
		t.Fatalf("expected len 2, got %d", r.Len())
	}
}

func TestRingVec3_SliceOrder(t *testing.T) {
	r := NewRingVec3(3)
	r.Push3(1, 2, 3)
	r.Push3(4, 5, 6)

	got := r.Slice()
	if len(got) != 2 {
		t.Fatalf("expected 2 elements, got %d", len(got))
	}
	if got[0].X != 1 || got[0].Y != 2 || got[0].Z != 3 {
		t.Errorf("got[0] = %+v, want {1 2 3}", got[0])
	}
	if got[1].X != 4 || got[1].Y != 5 || got[1].Z != 6 {
		t.Errorf("got[1] = %+v, want {4 5 6}", got[1])
	}
}

func TestRingVec3_SliceAfterWrap(t *testing.T) {
	r := NewRingVec3(2)
	r.Push3(1, 1, 1)
	r.Push3(2, 2, 2)
	r.Push3(3, 3, 3) // overwrites first

	got := r.Slice()
	if len(got) != 2 {
		t.Fatalf("expected 2 elements, got %d", len(got))
	}
	if got[0].X != 2 {
		t.Errorf("got[0].X = %f, want 2", got[0].X)
	}
	if got[1].X != 3 {
		t.Errorf("got[1].X = %f, want 3", got[1].X)
	}
}
