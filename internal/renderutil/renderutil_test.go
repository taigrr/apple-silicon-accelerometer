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

func TestDownsampleNonPositiveWidth(t *testing.T) {
	got := Downsample([]float64{1, 2, 3}, 0)
	if got != nil {
		t.Fatalf("Downsample() = %v, want nil", got)
	}

	got = Downsample([]float64{1, 2, 3}, -1)
	if got != nil {
		t.Fatalf("Downsample() with negative width = %v, want nil", got)
	}
}

func TestSparklineNonPositiveWidth(t *testing.T) {
	got := Sparkline([]float64{1, 2, 3}, 0, 0)
	if got != "" {
		t.Fatalf("Sparkline() = %q, want empty string", got)
	}

	got = Sparkline(nil, -1, 0)
	if got != "" {
		t.Fatalf("Sparkline() with negative width = %q, want empty string", got)
	}
}

func TestVisibleLenIgnoresANSIEscapeCodes(t *testing.T) {
	input := "\x1b[31malert\x1b[0m ok"
	if got, want := VisibleLen(input), len("alert ok"); got != want {
		t.Fatalf("VisibleLen() = %d, want %d", got, want)
	}
}

func TestGaugeNonPositiveWidth(t *testing.T) {
	got := Gauge(0, -1, 1, 0)
	if got != "" {
		t.Fatalf("Gauge() = %q, want empty string", got)
	}

	got = Gauge(0, -1, 1, -1)
	if got != "" {
		t.Fatalf("Gauge() with negative width = %q, want empty string", got)
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
