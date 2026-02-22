//go:build darwin

// Package sensor reads accelerometer, gyroscope, ambient light, and lid angle
// from Apple Silicon MacBooks via IOKit HID (AppleSPUHIDDevice, Bosch BMI286 IMU).
package sensor

// HID usage pages and usages for Apple SPU sensors.
const (
	PageVendor = 0xFF00 // Apple vendor page
	PageSensor = 0x0020 // HID sensor page
	UsageAccel = 3      // Accelerometer
	UsageGyro  = 9      // Gyroscope
	UsageALS   = 4      // Ambient light sensor
	UsageLid   = 138    // Lid angle sensor
)

// Report format constants for the Bosch BMI286 IMU.
const (
	IMUReportLen  = 22   // Accel/gyro report length in bytes
	IMUDecimation = 8    // Keep 1 in N samples
	IMUDataOffset = 6    // XYZ payload start offset
	ALSReportLen  = 122  // ALS report length in bytes
	LidReportLen  = 3    // Lid angle report length in bytes
	ReportBufSize = 4096 // HID callback buffer size
	ReportIntervalUS = 1000 // Driver report interval in microseconds
)

// CoreFoundation type IDs.
const (
	CFStringEncodingUTF8 = 0x08000100
	CFNumberSInt32Type   = 3
	CFNumberSInt64Type   = 4
)
