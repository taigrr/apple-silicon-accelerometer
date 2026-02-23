// spank detects slaps/hits on the laptop and plays audio responses.
package main

import (
	"bytes"
	"context"
	"embed"
	"fmt"
	"io"
	"math/rand"
	"os"
	"os/signal"
	"sort"
	"sync"
	"syscall"
	"time"

	"github.com/charmbracelet/fang"
	"github.com/gopxl/beep/v2"
	"github.com/gopxl/beep/v2/mp3"
	"github.com/gopxl/beep/v2/speaker"
	"github.com/spf13/cobra"
	"github.com/taigrr/apple-silicon-accelerometer/detector"
	"github.com/taigrr/apple-silicon-accelerometer/shm"
)

var version = "dev"

//go:embed audio/pain/*.mp3
var painAudio embed.FS

//go:embed audio/sexy/*.mp3
var sexyAudio embed.FS

var sexyMode bool

func main() {
	cmd := &cobra.Command{
		Use:   "spank",
		Short: "Yells 'ow!' when you slap the laptop",
		Long: `spank reads accelerometer data from shared memory (created by sensord)
and plays audio responses when a slap or hit is detected.

Requires:
  - sensord running (with sudo) to provide sensor data

Use --sexy for a different experience. In sexy mode, the more you slap
within a minute, the more intense the sounds become.`,
		Version: version,
		RunE: func(cmd *cobra.Command, args []string) error {
			return run(cmd.Context())
		},
		SilenceUsage: true,
	}

	cmd.Flags().BoolVarP(&sexyMode, "sexy", "s", false, "Enable sexy mode")

	if err := fang.Execute(context.Background(), cmd); err != nil {
		os.Exit(1)
	}
}

type slapTracker struct {
	mu        sync.Mutex
	times     []time.Time
	window    time.Duration
	sexyFiles []string
	altIdx    int
}

func newSlapTracker() *slapTracker {
	entries, _ := sexyAudio.ReadDir("audio/sexy")
	files := make([]string, 0, len(entries))
	for _, e := range entries {
		if !e.IsDir() {
			files = append(files, "audio/sexy/"+e.Name())
		}
	}
	sort.Strings(files)
	return &slapTracker{
		window:    time.Minute,
		sexyFiles: files,
	}
}

func (st *slapTracker) record(t time.Time) int {
	st.mu.Lock()
	defer st.mu.Unlock()

	cutoff := t.Add(-st.window)
	newTimes := make([]time.Time, 0, len(st.times)+1)
	for _, tt := range st.times {
		if tt.After(cutoff) {
			newTimes = append(newTimes, tt)
		}
	}
	newTimes = append(newTimes, t)
	st.times = newTimes
	return len(st.times)
}

func (st *slapTracker) getSexyFile(count int) string {
	st.mu.Lock()
	defer st.mu.Unlock()

	if len(st.sexyFiles) == 0 {
		return ""
	}

	maxIdx := len(st.sexyFiles) - 1
	topTwo := maxIdx - 1
	if topTwo < 0 {
		topTwo = 0
	}

	var idx int
	if count >= 20 {
		st.altIdx = 1 - st.altIdx
		idx = topTwo + st.altIdx
	} else {
		ratio := float64(count) / 20.0
		if ratio > 1 {
			ratio = 1
		}
		idx = int(ratio * float64(topTwo))
	}

	if idx > maxIdx {
		idx = maxIdx
	}
	return st.sexyFiles[idx]
}

func run(ctx context.Context) error {
	ctx, cancel := signal.NotifyContext(ctx, syscall.SIGINT, syscall.SIGTERM)
	defer cancel()

	accelRing, err := shm.OpenRing(shm.NameAccel)
	if err != nil {
		return fmt.Errorf("opening accel shm (is sensord running?): %w", err)
	}
	defer accelRing.Close()

	painFiles, err := loadPainFiles()
	if err != nil {
		return fmt.Errorf("loading pain audio: %w", err)
	}

	tracker := newSlapTracker()
	speakerInit := false
	det := detector.New()
	var lastAccelTotal uint64
	lastYell := time.Time{}
	cooldown := 500 * time.Millisecond
	maxBatch := 200

	mode := "pain"
	if sexyMode {
		mode = "sexy"
	}
	fmt.Printf("spank: listening for slaps in %s mode... (ctrl+c to quit)\n", mode)

	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			fmt.Println("\nbye!")
			return nil
		case <-ticker.C:
		}

		now := time.Now()
		tNow := float64(now.UnixNano()) / 1e9

		samples, newTotal := accelRing.ReadNew(lastAccelTotal, shm.AccelScale)
		lastAccelTotal = newTotal
		if len(samples) > maxBatch {
			samples = samples[len(samples)-maxBatch:]
		}

		prevEventCount := len(det.Events)
		nSamples := len(samples)
		for idx, s := range samples {
			tSample := tNow - float64(nSamples-idx-1)/float64(det.FS)
			det.Process(s.X, s.Y, s.Z, tSample)
		}

		if len(det.Events) > prevEventCount && time.Since(lastYell) > cooldown {
			ev := det.Events[len(det.Events)-1]
			if ev.Severity == "CHOC_MAJEUR" || ev.Severity == "CHOC_MOYEN" || ev.Severity == "MICRO_CHOC" {
				lastYell = now
				count := tracker.record(now)

				if sexyMode {
					file := tracker.getSexyFile(count)
					fmt.Printf("slap #%d [%s amp=%.5fg] -> %s\n", count, ev.Severity, ev.Amplitude, file)
					go playEmbedded(sexyAudio, file, &speakerInit)
				} else {
					file := painFiles[rand.Intn(len(painFiles))]
					fmt.Printf("ouch! [%s amp=%.5fg]\n", ev.Severity, ev.Amplitude)
					go playEmbedded(painAudio, file, &speakerInit)
				}
			}
		}
	}
}

func loadPainFiles() ([]string, error) {
	entries, err := painAudio.ReadDir("audio/pain")
	if err != nil {
		return nil, err
	}
	files := make([]string, 0, len(entries))
	for _, e := range entries {
		if !e.IsDir() {
			files = append(files, "audio/pain/"+e.Name())
		}
	}
	return files, nil
}

var (
	speakerMu sync.Mutex
)

func playEmbedded(fs embed.FS, path string, speakerInit *bool) {
	data, err := fs.ReadFile(path)
	if err != nil {
		return
	}

	streamer, format, err := mp3.Decode(io.NopCloser(bytes.NewReader(data)))
	if err != nil {
		return
	}
	defer streamer.Close()

	speakerMu.Lock()
	if !*speakerInit {
		speaker.Init(format.SampleRate, format.SampleRate.N(time.Second/10))
		*speakerInit = true
	}
	speakerMu.Unlock()

	done := make(chan bool)
	speaker.Play(beep.Seq(streamer, beep.Callback(func() {
		done <- true
	})))
	<-done
}
