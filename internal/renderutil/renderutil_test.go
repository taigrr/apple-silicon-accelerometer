package renderutil

import (
	"reflect"
	"testing"
)

func TestDownsamplePreservesLargestMagnitudePerBucket(t *testing.T) {
	data := []float64{-0.9, 0.1, 0.2, -0.8, 0.3, 0.4}
	got := Downsample(data, 3)
	want := []float64{-0.9, -0.8, 0.4}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("Downsample() = %v, want %v", got, want)
	}
}

func TestVisibleLenIgnoresANSIEscapeCodes(t *testing.T) {
	input := "\x1b[31malert\x1b[0m ok"
	if got, want := VisibleLen(input), len("alert ok"); got != want {
		t.Fatalf("VisibleLen() = %d, want %d", got, want)
	}
}

func TestGaugePlacesMarkerAtClampedPosition(t *testing.T) {
	got := Gauge(2, -1, 1, 5)
	want := "──┼─●"
	if got != want {
		t.Fatalf("Gauge() = %q, want %q", got, want)
	}

	got = Gauge(-2, -1, 1, 5)
	want = "●─┼──"
	if got != want {
		t.Fatalf("Gauge() with low clamp = %q, want %q", got, want)
	}
}
