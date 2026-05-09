// ============================================================
// PX4 ULog — InfluxDB Flux Query Reference
// Bucket: px4_flight_logs  |  Org: aerobatics
// ============================================================
// Usage: paste any block into the Data Explorer Script Editor.
// Change the log_file filter to target a specific flight.
// ============================================================


// ------------------------------------------------------------
// SANITY CHECKS
// ------------------------------------------------------------

// List all measurements in the bucket
import "influxdata/influxdb/schema"
schema.measurements(bucket: "px4_flight_logs")

// ---

// List all ingested log files
import "influxdata/influxdb/schema"
schema.tagValues(bucket: "px4_flight_logs", tag: "log_file")

// ---

// Quick data check — first 10 rows of anything
from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> limit(n: 10)


// ------------------------------------------------------------
// ALTITUDE  [Visualization: Graph]
// NED frame: z is positive-down, so negate for "altitude up"
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "vehicle_local_position" and r._field == "z")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")
  |> map(fn: (r) => ({r with _value: -r._value}))


// ------------------------------------------------------------
// XY GROUND TRACK  [Visualization: Scatter Plot]
// After running: set X axis = "y" (East), Y axis = "x" (North)
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "vehicle_local_position")
  |> filter(fn: (r) => r._field == "x" or r._field == "y")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")


// ------------------------------------------------------------
// BATTERY  [Visualization: Graph + Single Stat]
// ------------------------------------------------------------

// Voltage (V)
from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "battery_status" and r._field == "voltage_v")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")

// ---

// Remaining capacity (0.0–1.0)
from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "battery_status" and r._field == "remaining")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")


// ------------------------------------------------------------
// IMU VIBRATION  [Visualization: Graph]
// High variance = vibration problem. Normal < 4.0 m²/s⁴
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "sensor_combined")
  |> filter(fn: (r) => r._field == "accelerometer_m_s2_0"
                    or r._field == "accelerometer_m_s2_1"
                    or r._field == "accelerometer_m_s2_2")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")


// ------------------------------------------------------------
// CPU LOAD  [Visualization: Graph]
// load: 0.0–1.0 (fraction). warn > 0.70, error > 0.90
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "cpuload")
  |> filter(fn: (r) => r._field == "load" or r._field == "ram_usage")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")


// ------------------------------------------------------------
// ANGULAR VELOCITY  [Visualization: Graph]
// Roll/pitch/yaw rates in rad/s
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "vehicle_angular_velocity")
  |> filter(fn: (r) => r._field == "xyz_0" or r._field == "xyz_1" or r._field == "xyz_2")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")


// ------------------------------------------------------------
// LOCAL POSITION (full)  [Visualization: Graph]
// x=North, y=East, z=Down (negate for altitude), vx/vy/vz
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "vehicle_local_position")
  |> filter(fn: (r) => r._field == "x" or r._field == "y" or r._field == "vx" or r._field == "vy")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")


// ------------------------------------------------------------
// EKF HEALTH  [Visualization: Graph]
// innovation failures show as 1.0; should stay 0.0
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "estimator_status")
  |> filter(fn: (r) => r._field == "vel_test_ratio"
                    or r._field == "pos_test_ratio"
                    or r._field == "hgt_test_ratio")
  |> filter(fn: (r) => r.log_file == "log_32_2024-8-3-13-05-30.ulg")


// ------------------------------------------------------------
// ALL FLIGHTS OVERVIEW — altitude across all logs  [Visualization: Graph]
// Each log_file appears as a separate series
// ------------------------------------------------------------

from(bucket: "px4_flight_logs")
  |> range(start: 2024-01-01T00:00:00Z)
  |> filter(fn: (r) => r._measurement == "vehicle_local_position" and r._field == "z")
  |> map(fn: (r) => ({r with _value: -r._value}))
