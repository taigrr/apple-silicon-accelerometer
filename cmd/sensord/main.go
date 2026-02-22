// sensord is a daemon that reads Apple Silicon IMU sensors and writes
// data to POSIX shared memory ring buffers for consumption by sensordash
// or other readers.
package main

import (
	"context"
	"fmt"
	"os"

	"github.com/charmbracelet/fang"
	"github.com/spf13/cobra"
	"github.com/taigrr/apple-silicon-accelerometer/sensor"
	"github.com/taigrr/apple-silicon-accelerometer/shm"
)

var version = "dev"

func main() {
	cmd := &cobra.Command{
		Use:   "sensord",
		Short: "Apple Silicon sensor daemon",
		Long: `sensord reads accelerometer, gyroscope, ambient light, and lid angle
from Apple Silicon MacBooks via IOKit HID and writes data to POSIX
shared memory for consumption by sensordash or other programs.

Requires root privileges (sudo).`,
		Version: version,
		RunE: func(cmd *cobra.Command, args []string) error {
			return run()
		},
		SilenceUsage: true,
	}

	if err := fang.Execute(context.Background(), cmd); err != nil {
		os.Exit(1)
	}
}

func run() error {
	if os.Geteuid() != 0 {
		return fmt.Errorf("sensord requires root privileges, run with: sudo sensord")
	}

	// Create shared memory segments.
	accelRing, err := shm.CreateRing(shm.NameAccel)
	if err != nil {
		return fmt.Errorf("creating accel shm: %w", err)
	}
	defer accelRing.Close()
	defer accelRing.Unlink()

	gyroRing, err := shm.CreateRing(shm.NameGyro)
	if err != nil {
		return fmt.Errorf("creating gyro shm: %w", err)
	}
	defer gyroRing.Close()
	defer gyroRing.Unlink()

	alsSnap, err := shm.CreateSnapshot(shm.NameALS, shm.ALSSize)
	if err != nil {
		return fmt.Errorf("creating ALS shm: %w", err)
	}
	defer alsSnap.Close()
	defer alsSnap.Unlink()

	lidSnap, err := shm.CreateSnapshot(shm.NameLid, shm.LidSize)
	if err != nil {
		return fmt.Errorf("creating lid shm: %w", err)
	}
	defer lidSnap.Close()
	defer lidSnap.Unlink()

	fmt.Println("sensord: reading Apple Silicon sensors (Ctrl+C to stop)")

	// Run blocks forever on the CFRunLoop.
	return sensor.Run(sensor.Config{
		AccelRing: accelRing,
		GyroRing:  gyroRing,
		ALSSnap:   alsSnap,
		LidSnap:   lidSnap,
		Restarts:  0,
	})
}
