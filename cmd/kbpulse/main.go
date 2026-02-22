// kbpulse controls Apple Silicon MacBook keyboard backlight brightness.
// It can set a static level, pulse/breathe, or stream intensity from stdin
// (for integration with sensord or other programs).
package main

import (
	"bufio"
	"context"
	"fmt"
	"math"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"syscall"
	"time"

	"github.com/charmbracelet/fang"
	"github.com/spf13/cobra"
	"github.com/taigrr/apple-silicon-accelerometer/keyboard"
)

var version = "dev"

var (
	fadeMs     int
	keyboardID uint64
)

func main() {
	root := &cobra.Command{
		Use:   "kbpulse",
		Short: "Control MacBook keyboard backlight",
		Long: `kbpulse controls the Apple Silicon MacBook keyboard backlight brightness
via the private CoreBrightness framework.

Modes:
  set <0.0-1.0>    Set brightness to a fixed level
  get              Get current brightness
  breathe          Smooth breathing animation
  stdin            Stream brightness values from stdin (one float per line)
  off              Turn off the backlight`,
		Version: version,
		SilenceUsage: true,
	}

	root.PersistentFlags().IntVar(&fadeMs, "fade-ms", 20, "fade duration in milliseconds")
	root.PersistentFlags().Uint64Var(&keyboardID, "keyboard", 1, "keyboard ID (1 = built-in)")

	root.AddCommand(
		setCmd(),
		getCmd(),
		breatheCmd(),
		stdinCmd(),
		offCmd(),
	)

	if err := fang.Execute(context.Background(), root); err != nil {
		os.Exit(1)
	}
}

func newClient() (*keyboard.Client, error) {
	return keyboard.NewClient(keyboardID)
}

func setCmd() *cobra.Command {
	return &cobra.Command{
		Use:   "set <brightness>",
		Short: "Set brightness (0.0–1.0)",
		Args:  cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			level, err := strconv.ParseFloat(args[0], 32)
			if err != nil {
				return fmt.Errorf("invalid brightness: %w", err)
			}
			level = math.Max(0, math.Min(1, level))

			client, err := newClient()
			if err != nil {
				return err
			}
			client.SetAutoBrightness(false)
			client.SuspendIdleDimming(true)
			client.SetBrightness(float32(level), fadeMs)
			fmt.Printf("%.2f\n", level)
			return nil
		},
	}
}

func getCmd() *cobra.Command {
	return &cobra.Command{
		Use:   "get",
		Short: "Get current brightness",
		RunE: func(cmd *cobra.Command, args []string) error {
			client, err := newClient()
			if err != nil {
				return err
			}
			fmt.Printf("%.4f\n", client.GetBrightness())
			return nil
		},
	}
}

func breatheCmd() *cobra.Command {
	var periodSec float64
	var minLevel float64
	var maxLevel float64

	cmd := &cobra.Command{
		Use:   "breathe",
		Short: "Smooth breathing animation",
		RunE: func(cmd *cobra.Command, args []string) error {
			ctx, cancel := signal.NotifyContext(cmd.Context(), syscall.SIGINT, syscall.SIGTERM)
			defer cancel()

			client, err := newClient()
			if err != nil {
				return err
			}
			client.SetAutoBrightness(false)
			client.SuspendIdleDimming(true)

			defer func() {
				client.SetBrightness(0, fadeMs)
				client.SuspendIdleDimming(false)
			}()

			ticker := time.NewTicker(16 * time.Millisecond) // ~60 FPS
			defer ticker.Stop()

			start := time.Now()
			for {
				select {
				case <-ctx.Done():
					return nil
				case <-ticker.C:
					elapsed := time.Since(start).Seconds()
					// Sinusoidal breathing: (sin(t) + 1) / 2 mapped to min–max
					phase := math.Sin(2*math.Pi*elapsed/periodSec)*0.5 + 0.5
					level := minLevel + phase*(maxLevel-minLevel)
					client.SetBrightness(float32(level), fadeMs)
				}
			}
		},
	}

	cmd.Flags().Float64Var(&periodSec, "period", 4.0, "breathing period in seconds")
	cmd.Flags().Float64Var(&minLevel, "min", 0.0, "minimum brightness")
	cmd.Flags().Float64Var(&maxLevel, "max", 1.0, "maximum brightness")

	return cmd
}

func stdinCmd() *cobra.Command {
	return &cobra.Command{
		Use:   "stdin",
		Short: "Stream brightness from stdin (one float per line, 0.0–1.0)",
		Long: `Reads brightness values from stdin, one per line.
Useful for piping sensor data or custom animations.
Send "quit" or "exit" to stop. Ctrl+C also works.`,
		RunE: func(cmd *cobra.Command, args []string) error {
			ctx, cancel := signal.NotifyContext(cmd.Context(), syscall.SIGINT, syscall.SIGTERM)
			defer cancel()

			client, err := newClient()
			if err != nil {
				return err
			}
			client.SetAutoBrightness(false)
			client.SuspendIdleDimming(true)

			defer func() {
				client.SetBrightness(0, fadeMs)
				client.SuspendIdleDimming(false)
			}()

			scanner := bufio.NewScanner(os.Stdin)
			for scanner.Scan() {
				select {
				case <-ctx.Done():
					return nil
				default:
				}

				line := strings.TrimSpace(scanner.Text())
				if line == "quit" || line == "exit" {
					return nil
				}

				level, err := strconv.ParseFloat(line, 32)
				if err != nil {
					continue
				}
				level = math.Max(0, math.Min(1, level))
				client.SetBrightness(float32(level), fadeMs)
			}
			return scanner.Err()
		},
	}
}

func offCmd() *cobra.Command {
	return &cobra.Command{
		Use:   "off",
		Short: "Turn off the backlight",
		RunE: func(cmd *cobra.Command, args []string) error {
			client, err := newClient()
			if err != nil {
				return err
			}
			client.SetBrightness(0, fadeMs)
			client.SuspendIdleDimming(false)
			client.SetAutoBrightness(true)
			return nil
		},
	}
}
