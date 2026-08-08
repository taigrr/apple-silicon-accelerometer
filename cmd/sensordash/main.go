//go:build darwin

// sensordash is a terminal dashboard that reads sensor data from shared
// memory (written by sensord) and displays live vibration detection,
// orientation, heartbeat estimation, and environmental sensors.
package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/charmbracelet/fang"
	"github.com/spf13/cobra"
	"github.com/taigrr/apple-silicon-accelerometer/detector"
	"github.com/taigrr/apple-silicon-accelerometer/internal/renderutil"
	"github.com/taigrr/apple-silicon-accelerometer/shm"
)

var version = "dev"

// ANSI escape codes.
const (
	rst     = "\x1b[0m"
	bold    = "\x1b[1m"
	dim     = "\x1b[2m"
	red     = "\x1b[31m"
	grn     = "\x1b[32m"
	yel     = "\x1b[33m"
	cyn     = "\x1b[36m"
	bred    = "\x1b[91m"
	bwht    = "\x1b[97m"
	hideCur = "\x1b[?25l"
	showCur = "\x1b[?25h"
	altOn   = "\x1b[?1049h"
	altOff  = "\x1b[?1049l"
	clear   = "\x1b[2J\x1b[H"

	width = 76
)

func main() {
	cmd := &cobra.Command{
		Use:   "sensordash",
		Short: "Live sensor dashboard for Apple Silicon",
		Long: `sensordash reads sensor data from shared memory (created by sensord)
and displays a live terminal dashboard with vibration detection,
orientation (Mahony AHRS), heartbeat estimation (BCG), lid angle,
and ambient light sensor readings.

Run sensord first in another terminal (with sudo).`,
		Version: version,
		RunE: func(cmd *cobra.Command, args []string) error {
			return run(cmd.Context())
		},
		SilenceUsage: true,
	}

	if err := fang.Execute(context.Background(), cmd); err != nil {
		os.Exit(1)
	}
}

func run(ctx context.Context) error {
	ctx, cancel := signal.NotifyContext(ctx, syscall.SIGINT, syscall.SIGTERM)
	defer cancel()

	// Open shared memory segments (read-only).
	accelRing, err := shm.OpenRing(shm.NameAccel)
	if err != nil {
		return fmt.Errorf("opening accel shm (is sensord running?): %w", err)
	}
	defer accelRing.Close()

	gyroRing, err := shm.OpenRing(shm.NameGyro)
	if err != nil {
		return fmt.Errorf("opening gyro shm: %w", err)
	}
	defer gyroRing.Close()

	alsSnap, err := shm.OpenSnapshot(shm.NameALS, shm.ALSSize)
	if err != nil {
		return fmt.Errorf("opening ALS shm: %w", err)
	}
	defer alsSnap.Close()

	lidSnap, err := shm.OpenSnapshot(shm.NameLid, shm.LidSize)
	if err != nil {
		return fmt.Errorf("opening lid shm: %w", err)
	}
	defer lidSnap.Close()

	det := detector.New()
	tStart := time.Now()
	var lastAccelTotal, lastGyroTotal uint64
	var lastALSCount, lastLidCount uint32
	var lidAngle float64
	lidValid := false
	var alsRaw []byte
	lastDraw := time.Time{}
	lastPeriod := time.Time{}
	maxBatch := 200

	fmt.Print(altOn + hideCur)
	defer fmt.Print(showCur + altOff + "\n")

	ticker := time.NewTicker(5 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
		}

		now := time.Now()
		tNow := float64(now.UnixNano()) / 1e9

		// Read accelerometer
		samples, newTotal := accelRing.ReadNew(lastAccelTotal, shm.AccelScale)
		lastAccelTotal = newTotal
		if len(samples) > maxBatch {
			samples = samples[len(samples)-maxBatch:]
		}
		nSamples := len(samples)
		for idx, s := range samples {
			tSample := tNow - float64(nSamples-idx-1)/float64(det.FS)
			det.Process(s.X, s.Y, s.Z, tSample)
		}

		// Read gyroscope
		gyroSamples, newGyroTotal := gyroRing.ReadNew(lastGyroTotal, shm.GyroScale)
		lastGyroTotal = newGyroTotal
		if len(gyroSamples) > maxBatch {
			gyroSamples = gyroSamples[len(gyroSamples)-maxBatch:]
		}
		for _, g := range gyroSamples {
			det.ProcessGyro(g.X, g.Y, g.Z)
		}

		// Read ALS
		alsData, newALSCount := alsSnap.Read(lastALSCount, shm.ALSReportLen)
		lastALSCount = newALSCount
		if alsData != nil {
			alsRaw = alsData
		}

		// Read lid
		angle, newLidCount, changed := lidSnap.ReadFloat32(lastLidCount)
		lastLidCount = newLidCount
		if changed {
			lidAngle = float64(angle)
			lidValid = true
		}

		// Periodic analysis
		if now.Sub(lastPeriod) >= time.Second {
			det.DetectPeriodicity()
			det.DetectHeartbeat()
			lastPeriod = now
		}

		// Draw at ~10 FPS
		if now.Sub(lastDraw) >= 100*time.Millisecond {
			frame := render(det, tStart, lidAngle, lidValid, alsRaw)
			fmt.Print(clear + frame)
			lastDraw = now
		}
	}
}

func render(det *detector.Detector, tStart time.Time, lidAngle float64, lidValid bool, alsRaw []byte) string {
	elapsed := time.Since(tStart).Seconds()
	rate := float64(det.SampleCount) / elapsed
	if elapsed < 1 {
		rate = 0
	}

	var b strings.Builder
	gw := width - 4

	line := func(content string) {
		vl := renderutil.VisibleLen(content)
		pad := max(0, width-vl)
		fmt.Fprintf(&b, "%s│%s%s%s│%s\n", dim, rst, content, strings.Repeat(" ", pad), rst)
	}
	sep := func(label string) {
		if label != "" {
			rest := width - renderutil.VisibleLen(label) - 1
			fmt.Fprintf(&b, "%s├─%s%s┤%s\n", dim, label, strings.Repeat("─", rest), rst)
		} else {
			fmt.Fprintf(&b, "%s├%s┤%s\n", dim, strings.Repeat("─", width), rst)
		}
	}

	// Header
	title := " MOTION DETECTOR "
	topBar := strings.Repeat("─", width-len(title)-1)
	fmt.Fprintf(&b, "%s┌─%s%s%s%s%s┐%s\n", dim, rst, bwht, title, rst, dim+topBar, rst)

	hdr := fmt.Sprintf(" %s%7.1fs%s  %10d smp  %s%.0f%s Hz  Ev:%d",
		dim, elapsed, rst, det.SampleCount, bwht, rate, rst, len(det.Events))
	line(hdr)

	// Waveform
	sep(" Waveform |a_dyn| 5s ")
	wav := det.Waveform.Slice()
	if len(wav) > 0 {
		mx := 0.0002
		for _, v := range wav {
			if math.Abs(v) > mx {
				mx = math.Abs(v)
			}
		}
		ds := renderutil.Downsample(wav, gw)
		line(fmt.Sprintf("  %s%s%s", grn, renderutil.Sparkline(ds, gw, mx), rst))
		line(fmt.Sprintf("  %s%.5fg%s%s0g%s", dim, mx, strings.Repeat(" ", gw-22), rst, rst))
	} else {
		line(fmt.Sprintf("  %swaiting...%s", dim, rst))
		line("")
	}

	// XYZ axes
	sep(" Axes X / Y / Z (5s) ")
	xyz := det.WaveformXYZ.Slice()
	aw := gw - 4
	if len(xyz) > 0 {
		xs := make([]float64, len(xyz))
		ys := make([]float64, len(xyz))
		zs := make([]float64, len(xyz))
		amx := 0.0001
		for i, v := range xyz {
			xs[i] = v.X
			ys[i] = v.Y
			zs[i] = v.Z
			for _, val := range []float64{v.X, v.Y, v.Z} {
				if math.Abs(val) > amx {
					amx = math.Abs(val)
				}
			}
		}
		line(fmt.Sprintf("  %sX%s %s%s", red, rst, renderutil.Sparkline(renderutil.Downsample(xs, aw), aw, amx), rst))
		line(fmt.Sprintf("  %sY%s %s%s", grn, rst, renderutil.Sparkline(renderutil.Downsample(ys, aw), aw, amx), rst))
		line(fmt.Sprintf("  %sZ%s %s%s", cyn, rst, renderutil.Sparkline(renderutil.Downsample(zs, aw), aw, amx), rst))
	} else {
		line(fmt.Sprintf("  %sX%s", dim, rst))
		line(fmt.Sprintf("  %sY%s", dim, rst))
		line(fmt.Sprintf("  %sZ%s", dim, rst))
	}

	// RMS trend
	sep(" RMS trend 10s ")
	rms := det.RMSTrend.Slice()
	if len(rms) > 0 {
		line(fmt.Sprintf("  %s%s%s", yel, renderutil.Sparkline(rms, gw, 0), rst))
	} else {
		line(fmt.Sprintf("  %saccumulating...%s", dim, rst))
	}

	// Detectors
	sep(" Detectors ")
	names := [3]string{"fast", "med ", "slow"}
	for i := range 3 {
		sp := renderutil.Sparkline(det.STALTARings[i].Slice(), 25, det.STALTAOn(i)*2)
		r := det.STALTALatest[i]
		thr := det.STALTAOn(i)
		mark := " "
		col := dim
		if r > thr {
			mark = "*"
			col = bred
		}
		var extra string
		switch i {
		case 0:
			extra = fmt.Sprintf("  K:%5.1f  CF:%5.1f", det.Kurtosis, det.Crest)
		case 1:
			extra = fmt.Sprintf("  CUSUM:%8.4f", det.CUSUMVal)
		case 2:
			extra = fmt.Sprintf("  RMS:%.5fg Pk:%.5fg", det.RMS, det.Peak)
		}
		line(fmt.Sprintf(" %sSTA %s%s %s%s%s %s%5.1f%s%s%s",
			dim, names[i], rst, yel, sp, rst, col, r, mark, rst, extra))
	}

	// Autocorrelation
	sep(" Autocorrelation (lag 0.05-2.5s) ")
	if len(det.ACorrRing) > 0 {
		acCeil := 0.05
		for _, v := range det.ACorrRing {
			if math.Abs(v)*1.2 > acCeil {
				acCeil = math.Abs(v) * 1.2
			}
		}
		line(fmt.Sprintf("  %s%s%s", cyn, renderutil.Sparkline(det.ACorrRing, gw, acCeil), rst))
	} else {
		line(fmt.Sprintf("  %saccumulating...%s", dim, rst))
	}

	// Pattern
	sep(" Pattern ")
	if det.PeriodValid && det.PeriodCV < 0.5 {
		reg := max(0, min(100, int((1.0-det.PeriodCV)*100)))
		line(fmt.Sprintf(" Period:%.3fs ±%.3f  Freq:%.2fHz  Reg:%d%%",
			det.Period, det.PeriodSTD, det.PeriodFreq, reg))
	} else {
		line(fmt.Sprintf(" %sno regular pattern detected%s", dim, rst))
	}
	line("")

	// Heartbeat
	sep(" Heartbeat BCG ")
	if det.HRValid && det.HRConf > 0.15 {
		conf := int(det.HRConf * 100)
		line(fmt.Sprintf(" %s♥ %s%s%.1f BPM%s   confidence: %d%%   band: 0.8-3Hz",
			bred, bred, bold, det.HRBPM, rst, conf))
	} else {
		line(fmt.Sprintf(" %sno heartbeat detected (rest wrists on laptop)%s", dim, rst))
	}
	line("")

	// Orientation
	sep(" Orientation ")
	orient := det.GetOrientation()
	ow := width - 18
	line(fmt.Sprintf(" %sRoll %s %s%s%s %+7.1f°", dim, rst, cyn, renderutil.Gauge(orient.Roll, -180, 180, ow), rst, orient.Roll))
	line(fmt.Sprintf(" %sPitch%s %s%s%s %+7.1f°", dim, rst, cyn, renderutil.Gauge(orient.Pitch, -90, 90, ow), rst, orient.Pitch))
	line(fmt.Sprintf(" %sYaw  %s %s%s%s %+7.1f°", dim, rst, cyn, renderutil.Gauge(orient.Yaw, -180, 180, ow), rst, orient.Yaw))
	line(fmt.Sprintf(" %sω: %+6.2f  %+6.2f  %+6.2f °/s%s",
		dim, det.GyroLatest[0], det.GyroLatest[1], det.GyroLatest[2], rst))

	// Lid angle
	sep(" Lid Angle ")
	if lidValid {
		line(fmt.Sprintf("  %s%.0f°%s", bwht, lidAngle, rst))
	} else {
		line(fmt.Sprintf("  %sno lid data%s", dim, rst))
	}

	// Ambient light
	sep(" Ambient Light ")
	if len(alsRaw) >= 44 {
		lux := math.Float32frombits(binary.LittleEndian.Uint32(alsRaw[40:44]))
		line(fmt.Sprintf("  %s%.3f%s %slux%s", bwht, lux, rst, dim, rst))
	} else {
		line(fmt.Sprintf("  %swaiting for ALS data...%s", dim, rst))
	}

	// Events
	sep(" Events ")
	evts := det.Events
	start := max(0, len(evts)-5)
	for i := len(evts) - 1; i >= start; i-- {
		ev := evts[i]
		col := sevColor(ev.Severity)
		line(fmt.Sprintf(" %s%s%s %s%s %-11s%s %.5fg",
			dim, ev.Time.Format("15:04:05.000"), rst, col, ev.Symbol, ev.Label, rst, ev.Amplitude))
	}
	for range max(0, 3-(len(evts)-start)) {
		line("")
	}

	// Footer
	sep("")
	ax, ay, az := det.LatestRaw[0], det.LatestRaw[1], det.LatestRaw[2]
	line(fmt.Sprintf(" X:%+10.6fg Y:%+10.6fg Z:%+10.6fg  |g|:%.6f",
		ax, ay, az, det.LatestMag))
	line(fmt.Sprintf(" %sctrl+c to quit%s", dim, rst))
	fmt.Fprintf(&b, "%s└%s┘%s\n", dim, strings.Repeat("─", width), rst)

	return b.String()
}

func sevColor(sev string) string {
	switch sev {
	case "CHOC_MAJEUR":
		return bred + bold
	case "CHOC_MOYEN":
		return red
	case "MICRO_CHOC":
		return cyn
	case "VIBRATION":
		return yel
	case "VIB_LEGERE":
		return grn
	default:
		return dim
	}
}
