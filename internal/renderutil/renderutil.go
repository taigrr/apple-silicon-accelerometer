package renderutil

import (
	"math"
	"strings"
)

const blocks = " ▁▂▃▄▅▆▇█"

func Sparkline(data []float64, width int, ceil float64) string {
	if width <= 0 {
		return ""
	}
	if len(data) == 0 {
		return strings.Repeat(" ", width)
	}
	d := data
	if len(d) < width {
		pad := make([]float64, width-len(d))
		d = append(pad, d...)
	} else if len(d) > width {
		d = d[len(d)-width:]
	}
	if ceil <= 0 {
		for _, value := range d {
			if math.Abs(value) > ceil {
				ceil = math.Abs(value)
			}
		}
	}
	if ceil <= 0 {
		ceil = 1
	}
	blk := []rune(blocks)
	var b strings.Builder
	for _, value := range d {
		frac := math.Min(1, math.Abs(value)/ceil)
		idx := min(8, int(frac*8))
		b.WriteRune(blk[idx])
	}
	return b.String()
}

func Gauge(value, vmin, vmax float64, width int) string {
	if width <= 0 {
		return ""
	}
	rng := vmax - vmin
	if rng == 0 {
		rng = 1
	}
	t := math.Max(0, math.Min(1, (value-vmin)/rng))
	pos := int(t * float64(width-1))
	center := int((0 - vmin) / rng * float64(width-1))
	bar := make([]rune, width)
	for idx := range bar {
		bar[idx] = '─'
	}
	if center >= 0 && center < width {
		bar[center] = '┼'
	}
	bar[max(0, min(width-1, pos))] = '●'
	return string(bar)
}

func Downsample(data []float64, width int) []float64 {
	if width <= 0 {
		return nil
	}
	n := len(data)
	if n <= width {
		return data
	}
	step := float64(n) / float64(width)
	out := make([]float64, width)
	for column := range width {
		startIdx := int(float64(column) * step)
		endIdx := int(float64(column+1) * step)
		selected := data[startIdx]
		for sampleIdx := startIdx + 1; sampleIdx < endIdx && sampleIdx < n; sampleIdx++ {
			if math.Abs(data[sampleIdx]) > math.Abs(selected) {
				selected = data[sampleIdx]
			}
		}
		out[column] = selected
	}
	return out
}

func VisibleLen(s string) int {
	n := 0
	inEsc := false
	for _, r := range s {
		if r == '\033' {
			inEsc = true
			continue
		}
		if inEsc {
			if r == 'm' {
				inEsc = false
			}
			continue
		}
		n++
	}
	return n
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}
