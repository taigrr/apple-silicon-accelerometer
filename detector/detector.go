// Package detector implements vibration detection, orientation tracking
// (Mahony AHRS), and experimental heartbeat (BCG) detection from IMU data.
package detector

import (
	"math"
	"time"
)

// SampleRate is the expected input sample rate in Hz.
const SampleRate = 100

// Event represents a detected vibration event.
type Event struct {
	Time      time.Time
	Severity  string
	Symbol    string
	Label     string
	Amplitude float64
	Sources   []string
	Bands     []string
}

// Orientation holds Euler angles derived from quaternion AHRS.
type Orientation struct {
	Roll, Pitch, Yaw float64 // degrees
}

// Detector processes accelerometer and gyroscope data to detect vibrations,
// compute orientation, and estimate heartbeat.
type Detector struct {
	SampleCount int
	FS          int

	// Latest raw values
	LatestRaw [3]float64
	LatestMag float64

	// High-pass filter for gravity removal
	hpAlpha   float64
	hpPrevRaw [3]float64
	hpPrevOut [3]float64
	hpReady   bool

	// Waveform history
	Waveform    *RingFloat
	WaveformXYZ *RingVec3

	// STA/LTA detector (3 timescales)
	sta          [3]float64
	lta          [3]float64
	staN         [3]int
	ltaN         [3]int
	staLTAOn     [3]float64
	staLTAOff    [3]float64
	STALTAActive [3]bool
	STALTALatest [3]float64
	STALTARings  [3]*RingFloat

	// CUSUM
	cusumPos float64
	cusumNeg float64
	cusumMu  float64
	cusumK   float64
	cusumH   float64
	CUSUMVal float64

	// Kurtosis
	kurtBuf  *RingFloat
	Kurtosis float64

	// Peak / MAD / crest factor
	peakBuf  *RingFloat
	Crest    float64
	RMS      float64
	Peak     float64
	MADSigma float64

	// RMS trend
	RMSTrend  *RingFloat
	rmsWindow *RingFloat

	// Events
	Events     []Event
	lastEvtT   float64

	// Gyro latest
	GyroLatest [3]float64

	// Mahony AHRS quaternion
	Q           [4]float64 // w, x, y, z
	mahonyKP    float64
	mahonyKI    float64
	mahonyErrI  [3]float64
	orientInit  bool

	// Heartbeat BCG
	hrHPAlpha  float64
	hrLPAlpha  float64
	hrHPPrevIn float64
	hrHPPrevOut float64
	hrLPPrev   float64
	hrBuf      *RingFloat
	HRBPM      float64
	HRConf     float64
	HRValid    bool

	// Periodicity
	Period    float64
	PeriodFreq float64
	PeriodCV  float64
	PeriodSTD float64
	PeriodValid bool
	ACorrRing []float64

	// Internal counters
	staDec  int
	kurtDec int
	rmsDec  int
}

// New creates a new Detector with default parameters.
func New() *Detector {
	fs := SampleRate
	n5 := fs * 5

	d := &Detector{
		FS:      fs,
		hpAlpha: 0.95,

		Waveform:    NewRingFloat(n5),
		WaveformXYZ: NewRingVec3(n5),

		staN:     [3]int{3, 15, 50},
		ltaN:     [3]int{100, 500, 2000},
		staLTAOn: [3]float64{3.0, 2.5, 2.0},
		staLTAOff: [3]float64{1.5, 1.3, 1.2},

		cusumK: 0.0005,
		cusumH: 0.01,

		kurtBuf:  NewRingFloat(100),
		Kurtosis: 3.0,

		peakBuf: NewRingFloat(200),
		Crest:   1.0,

		RMSTrend:  NewRingFloat(100),
		rmsWindow: NewRingFloat(fs),

		Q:        [4]float64{1, 0, 0, 0},
		mahonyKP: 1.0,
		mahonyKI: 0.05,

		hrHPAlpha: float64(fs) / (float64(fs) + 2.0*math.Pi*0.8),
		hrLPAlpha: 2.0 * math.Pi * 3.0 / (2.0*math.Pi*3.0 + float64(fs)),
		hrBuf:     NewRingFloat(fs * 10),
	}

	for i := range 3 {
		d.lta[i] = 1e-10
		d.STALTALatest[i] = 1.0
		d.STALTARings[i] = NewRingFloat(30)
	}

	return d
}

// ProcessGyro updates the latest gyroscope reading (deg/s).
func (d *Detector) ProcessGyro(gx, gy, gz float64) {
	d.GyroLatest = [3]float64{gx, gy, gz}
}

// Process ingests one accelerometer sample and returns the dynamic magnitude.
func (d *Detector) Process(ax, ay, az, tNow float64) float64 {
	d.SampleCount++
	d.LatestRaw = [3]float64{ax, ay, az}
	d.LatestMag = math.Sqrt(ax*ax + ay*ay + az*az)
	d.updateOrientation(ax, ay, az)

	if !d.hpReady {
		d.hpPrevRaw = [3]float64{ax, ay, az}
		d.hpReady = true
		d.Waveform.Push(0)
		return 0
	}

	a := d.hpAlpha
	hx := a * (d.hpPrevOut[0] + ax - d.hpPrevRaw[0])
	hy := a * (d.hpPrevOut[1] + ay - d.hpPrevRaw[1])
	hz := a * (d.hpPrevOut[2] + az - d.hpPrevRaw[2])
	d.hpPrevRaw = [3]float64{ax, ay, az}
	d.hpPrevOut = [3]float64{hx, hy, hz}
	mag := math.Sqrt(hx*hx + hy*hy + hz*hz)

	d.Waveform.Push(mag)
	d.WaveformXYZ.Push3(hx, hy, hz)

	// Heartbeat bandpass
	hpOut := d.hrHPAlpha * (d.hrHPPrevOut + mag - d.hrHPPrevIn)
	d.hrHPPrevIn = mag
	d.hrHPPrevOut = hpOut
	lpOut := d.hrLPAlpha*hpOut + (1.0-d.hrLPAlpha)*d.hrLPPrev
	d.hrLPPrev = lpOut
	d.hrBuf.Push(lpOut)

	// RMS trend
	d.rmsWindow.Push(mag)
	d.rmsDec++
	if d.rmsDec >= max(1, d.FS/10) {
		d.rmsDec = 0
		vals := d.rmsWindow.Slice()
		if len(vals) > 0 {
			var s float64
			for _, v := range vals {
				s += v * v
			}
			d.RMSTrend.Push(math.Sqrt(s / float64(len(vals))))
		}
	}

	var evts []detection

	// STA/LTA
	e := mag * mag
	for i := range 3 {
		d.sta[i] += (e - d.sta[i]) / float64(d.staN[i])
		d.lta[i] += (e - d.lta[i]) / float64(d.ltaN[i])
		ratio := d.sta[i] / (d.lta[i] + 1e-30)
		d.STALTALatest[i] = ratio
		was := d.STALTAActive[i]
		if ratio > d.staLTAOn[i] && !was {
			d.STALTAActive[i] = true
			evts = append(evts, detection{source: "STA/LTA"})
		} else if ratio < d.staLTAOff[i] {
			d.STALTAActive[i] = false
		}
	}
	d.staDec++
	if d.staDec >= max(1, d.FS/30) {
		d.staDec = 0
		for i := range 3 {
			d.STALTARings[i].Push(d.STALTALatest[i])
		}
	}

	// CUSUM
	d.cusumMu += 0.0001 * (mag - d.cusumMu)
	d.cusumPos = math.Max(0, d.cusumPos+mag-d.cusumMu-d.cusumK)
	d.cusumNeg = math.Max(0, d.cusumNeg-mag+d.cusumMu-d.cusumK)
	d.CUSUMVal = math.Max(d.cusumPos, d.cusumNeg)
	if d.cusumPos > d.cusumH {
		evts = append(evts, detection{source: "CUSUM"})
		d.cusumPos = 0
	}
	if d.cusumNeg > d.cusumH {
		evts = append(evts, detection{source: "CUSUM"})
		d.cusumNeg = 0
	}

	// Kurtosis
	d.kurtBuf.Push(mag)
	d.kurtDec++
	if d.kurtDec >= 10 && d.kurtBuf.Len() >= 50 {
		d.kurtDec = 0
		buf := d.kurtBuf.Slice()
		n := float64(len(buf))
		mu := sum(buf) / n
		var m2, m4 float64
		for _, v := range buf {
			diff := v - mu
			d2 := diff * diff
			m2 += d2
			m4 += d2 * d2
		}
		m2 /= n
		m4 /= n
		d.Kurtosis = m4 / (m2*m2 + 1e-30)
		if d.Kurtosis > 6 {
			evts = append(evts, detection{source: "KURTOSIS"})
		}
	}

	// Peak / MAD
	d.peakBuf.Push(mag)
	if d.peakBuf.Len() >= 50 && d.SampleCount%10 == 0 {
		buf := d.peakBuf.Slice()
		sorted := sortedCopy(buf)
		n := len(sorted)
		median := sorted[n/2]

		devs := make([]float64, n)
		for i, v := range sorted {
			devs[i] = math.Abs(v - median)
		}
		sortFloat64s(devs)
		mad := devs[n/2]
		sigma := 1.4826*mad + 1e-30
		d.MADSigma = sigma

		var s float64
		var pk float64
		for _, v := range buf {
			s += v * v
			if math.Abs(v) > pk {
				pk = math.Abs(v)
			}
		}
		d.RMS = math.Sqrt(s / float64(n))
		d.Peak = pk
		d.Crest = pk / (d.RMS + 1e-30)

		dev := math.Abs(mag-median) / sigma
		if dev > 2.0 {
			evts = append(evts, detection{source: "PEAK"})
		}
	}

	if len(evts) > 0 && (tNow-d.lastEvtT) > 0.01 {
		d.lastEvtT = tNow
		d.classify(evts, tNow, mag)
	}

	return mag
}

// GetOrientation returns the current Euler angles from the AHRS quaternion.
func (d *Detector) GetOrientation() Orientation {
	qw, qx, qy, qz := d.Q[0], d.Q[1], d.Q[2], d.Q[3]

	sinR := 2.0 * (qw*qx + qy*qz)
	cosR := 1.0 - 2.0*(qx*qx+qy*qy)
	roll := math.Atan2(sinR, cosR) * 180 / math.Pi

	sinP := 2.0 * (qw*qy - qz*qx)
	sinP = math.Max(-1, math.Min(1, sinP))
	pitch := math.Asin(sinP) * 180 / math.Pi

	sinY := 2.0 * (qw*qz + qx*qy)
	cosY := 1.0 - 2.0*(qy*qy+qz*qz)
	yaw := math.Atan2(sinY, cosY) * 180 / math.Pi

	return Orientation{Roll: roll, Pitch: pitch, Yaw: yaw}
}

// DetectPeriodicity computes autocorrelation to find periodic vibration patterns.
func (d *Detector) DetectPeriodicity() {
	wav := d.Waveform.Slice()
	if len(wav) < d.FS*2 {
		d.PeriodValid = false
		d.ACorrRing = nil
		return
	}

	// Use last 5 seconds
	n := len(wav)
	if n > d.FS*5 {
		wav = wav[n-d.FS*5:]
		n = len(wav)
	}

	mean := sum(wav) / float64(n)
	centered := make([]float64, n)
	var variance float64
	for i, v := range wav {
		c := v - mean
		centered[i] = c
		variance += c * c
	}
	if variance < 1e-20 {
		d.PeriodValid = false
		d.ACorrRing = nil
		return
	}

	minLag := max(5, d.FS/20) // 0.05s
	maxLag := min(n/2, d.FS*5/2) // 2.5s

	acorr := make([]float64, 0, maxLag-minLag)
	for lag := minLag; lag < maxLag; lag++ {
		var s float64
		for i := range n - lag {
			s += centered[i] * centered[i+lag]
		}
		acorr = append(acorr, s/variance)
	}
	d.ACorrRing = acorr

	if len(acorr) == 0 {
		d.PeriodValid = false
		return
	}

	bestIdx := 0
	bestVal := acorr[0]
	for i, v := range acorr {
		if v > bestVal {
			bestVal = v
			bestIdx = i
		}
	}

	bestLag := minLag + bestIdx
	if bestVal > 0.1 {
		d.Period = float64(bestLag) / float64(d.FS)
		d.PeriodFreq = float64(d.FS) / float64(bestLag)
		d.PeriodCV = math.Max(0, 1.0-bestVal)
		d.PeriodSTD = d.Period * d.PeriodCV
		d.PeriodValid = true
	} else {
		d.PeriodValid = false
	}
}

// DetectHeartbeat uses autocorrelation of the bandpassed signal to estimate BPM.
func (d *Detector) DetectHeartbeat() {
	minN := d.FS * 5
	if d.hrBuf.Len() < minN {
		d.HRValid = false
		d.HRBPM = 0
		d.HRConf = 0
		return
	}

	buf := d.hrBuf.Slice()
	n := len(buf)
	if n > d.FS*10 {
		buf = buf[n-d.FS*10:]
		n = len(buf)
	}

	mean := sum(buf) / float64(n)
	centered := make([]float64, n)
	var variance float64
	for i, v := range buf {
		c := v - mean
		centered[i] = c
		variance += c * c
	}
	if variance < 1e-20 {
		d.HRValid = false
		return
	}

	lagLo := int(float64(d.FS) * 0.3) // ~200 BPM
	lagHi := min(int(float64(d.FS)*1.0), n/2) // ~60 BPM
	if lagLo >= lagHi {
		d.HRValid = false
		return
	}

	bestR := -1.0
	bestLag := lagLo
	for lag := lagLo; lag < lagHi; lag++ {
		var s float64
		for i := range n - lag {
			s += centered[i] * centered[i+lag]
		}
		r := s / variance
		if r > bestR {
			bestR = r
			bestLag = lag
		}
	}

	if bestR > 0.15 {
		d.HRBPM = 60.0 / (float64(bestLag) / float64(d.FS))
		d.HRConf = math.Min(1, bestR)
		d.HRValid = true
	} else {
		d.HRValid = false
	}
}

// updateOrientation runs the Mahony AHRS filter.
func (d *Detector) updateOrientation(ax, ay, az float64) {
	aNorm := math.Sqrt(ax*ax + ay*ay + az*az)
	if aNorm < 0.3 {
		return
	}

	gx := d.GyroLatest[0] * math.Pi / 180
	gy := d.GyroLatest[1] * math.Pi / 180
	gz := d.GyroLatest[2] * math.Pi / 180
	dt := 1.0 / float64(d.FS)

	if !d.orientInit {
		inv := 1.0 / aNorm
		axN, azN := ax*inv, az*inv
		ayN := ay * inv
		pitch0 := math.Atan2(-axN, -azN)
		roll0 := math.Atan2(ayN, -azN)
		cp := math.Cos(pitch0 * 0.5)
		sp := math.Sin(pitch0 * 0.5)
		cr := math.Cos(roll0 * 0.5)
		sr := math.Sin(roll0 * 0.5)
		d.Q = [4]float64{cr * cp, sr * cp, cr * sp, -sr * sp}
		d.orientInit = true
		return
	}

	qw, qx, qy, qz := d.Q[0], d.Q[1], d.Q[2], d.Q[3]
	inv := 1.0 / aNorm
	axN, ayN, azN := ax*inv, ay*inv, az*inv

	// Estimated gravity direction from quaternion
	vx := 2.0 * (qx*qz - qw*qy)
	vy := 2.0 * (qw*qx + qy*qz)
	vz := qw*qw - qx*qx - qy*qy + qz*qz

	// Cross product: measured × estimated (negative gravity)
	ex := ayN*(-vz) - azN*(-vy)
	ey := azN*(-vx) - axN*(-vz)
	ez := axN*(-vy) - ayN*(-vx)

	// PI correction
	d.mahonyErrI[0] += d.mahonyKI * ex * dt
	d.mahonyErrI[1] += d.mahonyKI * ey * dt
	d.mahonyErrI[2] += d.mahonyKI * ez * dt

	gx += d.mahonyKP*ex + d.mahonyErrI[0]
	gy += d.mahonyKP*ey + d.mahonyErrI[1]
	gz += d.mahonyKP*ez + d.mahonyErrI[2]

	// Quaternion derivative integration
	hdt := 0.5 * dt
	dw := (-qx*gx - qy*gy - qz*gz) * hdt
	dx := (qw*gx + qy*gz - qz*gy) * hdt
	dy := (qw*gy - qx*gz + qz*gx) * hdt
	dz := (qw*gz + qx*gy - qy*gx) * hdt

	qw += dw
	qx += dx
	qy += dy
	qz += dz

	// Normalize
	n := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
	if n > 0 {
		inv := 1.0 / n
		qw *= inv
		qx *= inv
		qy *= inv
		qz *= inv
	}

	d.Q = [4]float64{qw, qx, qy, qz}
}

type detection struct {
	source string
}

func (d *Detector) classify(dets []detection, t, amp float64) {
	sources := make(map[string]bool)
	for _, det := range dets {
		sources[det.source] = true
	}
	ns := len(sources)

	var sev, sym, lbl string
	switch {
	case ns >= 4 && amp > 0.05:
		sev, sym, lbl = "CHOC_MAJEUR", "★", "MAJOR"
	case ns >= 3 && amp > 0.02:
		sev, sym, lbl = "CHOC_MOYEN", "▲", "shock"
	case sources["PEAK"] && amp > 0.005:
		sev, sym, lbl = "MICRO_CHOC", "△", "micro-choc"
	case (sources["STA/LTA"] || sources["CUSUM"]) && amp > 0.003:
		sev, sym, lbl = "VIBRATION", "●", "vibration"
	case amp > 0.001:
		sev, sym, lbl = "VIB_LEGERE", "○", "light-vib"
	default:
		sev, sym, lbl = "MICRO_VIB", "·", "micro-vib"
	}

	srcList := make([]string, 0, len(sources))
	for s := range sources {
		srcList = append(srcList, s)
	}

	ev := Event{
		Time:      time.Unix(int64(t), int64((t-math.Floor(t))*1e9)),
		Severity:  sev,
		Symbol:    sym,
		Label:     lbl,
		Amplitude: amp,
		Sources:   srcList,
	}

	d.Events = append(d.Events, ev)
	// Keep max 500 events
	if len(d.Events) > 500 {
		d.Events = d.Events[len(d.Events)-500:]
	}
}

// STALTAOn returns the STA/LTA on-threshold for the given timescale index.
func (d *Detector) STALTAOn(i int) float64 {
	return d.staLTAOn[i]
}

// Helpers

func sum(s []float64) float64 {
	var t float64
	for _, v := range s {
		t += v
	}
	return t
}

func sortedCopy(s []float64) []float64 {
	c := make([]float64, len(s))
	copy(c, s)
	sortFloat64s(c)
	return c
}

func sortFloat64s(s []float64) {
	// Simple insertion sort — small slices only (<=200)
	for i := 1; i < len(s); i++ {
		key := s[i]
		j := i - 1
		for j >= 0 && s[j] > key {
			s[j+1] = s[j]
			j--
		}
		s[j+1] = key
	}
}
