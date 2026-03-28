# drone_diagnostics

ROS2 Humble package that monitors PX4 telemetry health in real time and publishes standardized
`diagnostic_msgs/DiagnosticArray` messages for display in `rqt_robot_monitor` or forwarding to
InfluxDB.

## System

| Component | Detail |
|-----------|--------|
| Flight controller | PX4 Autopilot via uXRCE-DDS bridge |
| ROS2 distribution | Humble (Ubuntu 22) |
| Drone namespace | `/pequi/hermit` (configurable) |
| Workspace | `~/ros2_humble` |

## Prerequisites

```bash
sudo apt install ros-humble-diagnostic-updater \
                 ros-humble-diagnostic-aggregator \
                 ros-humble-rqt-robot-monitor
```

## Package layout

```
drone_diagnostics/
├── drone_diagnostics/
│   ├── diagnostic_node.py       # 11 health checks → /diagnostics at 2 Hz
│   └── diagnostics_logger_node.py  # optional InfluxDB writer
├── config/
│   └── diagnostics_config.yaml  # aggregator groups + tunable thresholds
├── launch/
│   └── drone_diagnostics.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

## Diagnostic pipeline

```
diagnostic_node  ──►  /diagnostics  ──►  diagnostic_aggregator  ──►  /diagnostics_agg
                                                                              │
                                                                    rqt_robot_monitor
                                                                  (optional) InfluxDB
```

## Health checks

All checks subscribe to `/fmu/out` topics (BEST_EFFORT + TRANSIENT_LOCAL QoS).
Each check function contains an explicit `# HEALTH CRITERIA` block in the source.

| Name in `/diagnostics` | Source topic | OK condition |
|------------------------|--------------|--------------|
| `PX4: Vehicle Status` | `vehicle_status` | failsafe=False, arming ∈ {DISARMED, ARMED} |
| `PX4: Failsafe Flags` | `failsafe_flags` | all 13 monitored flags False |
| `PX4: Land Detected` | `vehicle_land_detected` | freefall=False |
| `PX4: CPU Load` | `cpuload` | cpu ≤ 70 % AND ram ≤ 80 % |
| `PX4: Time Sync` | `timesync_status` | \|offset\| ≤ 500 µs |
| `PX4: Battery Health` | `battery_status` | voltage ≥ 14.4 V AND remaining ≥ 25 % |
| `PX4: IMU (sensor_combined)` | `sensor_combined` | rate ≥ 50 Hz |
| `PX4: Angular Velocity` | `vehicle_angular_velocity` | rate ≥ 50 Hz |
| `PX4: Local Position` | `vehicle_local_position` | xy_valid=True AND z_valid=True |
| `PX4: EKF Health` | `estimator_status` | no innovation failures, horiz_accuracy ≤ 5 m |
| `PX4: IMU Vibration` | `sensor_combined` (derived) | max accel variance ≤ 4.0 m²/s⁴ |

### Status levels

| Level | Byte | Meaning |
|-------|------|---------|
| OK | `\x00` | Within all thresholds |
| WARN | `\x01` | Degraded but operational |
| ERROR | `\x02` | Out of bounds or topic absent |

Topics that have never been received return **WARN** for slow/status topics and **ERROR**
for high-rate topics (IMU, position, EKF).

## Parameters

Tunable at launch without rebuilding. Defaults are in `config/diagnostics_config.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_namespace` | `/pequi/hermit` | Namespace prefix for PX4 topics |
| `topic_timeout_sec` | `2.0` | Seconds until a topic is declared stale |
| `publish_rate_hz` | `2.0` | `/diagnostics` publish rate |
| `cpu_warn_pct` | `0.70` | CPU warn threshold (fraction) |
| `cpu_error_pct` | `0.90` | CPU error threshold (fraction) |
| `ram_warn_pct` | `0.80` | RAM error threshold (fraction) |
| `timesync_warn_us` | `500` | Clock offset warn (µs) |
| `timesync_error_us` | `2000` | Clock offset error (µs) |

## Build

```bash
# Package lives in ~/ros2_humble/src/drone_diagnostics
# After editing sources in this repo, sync first:
cp drone_diagnostics/drone_diagnostics/diagnostic_node.py \
   ~/ros2_humble/src/drone_diagnostics/drone_diagnostics/

cd ~/ros2_humble
source install/setup.bash
colcon build --packages-select drone_diagnostics
source install/setup.bash
```

## Run

```bash
# Node + aggregator (standard)
ros2 launch drone_diagnostics drone_diagnostics.launch.py

# Different drone
ros2 launch drone_diagnostics drone_diagnostics.launch.py drone_namespace:=/my_drone

# With InfluxDB logger
ros2 launch drone_diagnostics drone_diagnostics.launch.py \
    enable_influxdb:=true \
    influxdb_url:=http://localhost:8086 \
    influxdb_org:=aerobatics \
    influxdb_bucket:=drone_diagnostics

# Node only (no aggregator)
ros2 run drone_diagnostics diagnostic_node
```

## Monitor

```bash
# Raw status array (one entry per check)
ros2 topic echo /diagnostics

# Aggregated by group (PX4)
ros2 topic echo /diagnostics_agg

# GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## Test with rosbag

```bash
cd ~/ros2_humble && source install/setup.bash

# Terminal 1
ros2 run drone_diagnostics diagnostic_node

# Terminal 2
BAGS=/mnt/316f1a60-da34-40af-9076-4b3ee9fb0ed1/Documents/PMec/Flying/Diagnostic/rosbags
ros2 bag play $BAGS/bag_with_px4 --clock        # 14 s
# ros2 bag play $BAGS/bag_with_px4_60 --clock   # 60 s

# Terminal 3
ros2 topic echo /diagnostics --once
```

### Known issue — CDR deserialization with existing bags

`bag_with_px4` and `bag_with_px4_60` were recorded with an older PX4 firmware version.
Three topics fail to deserialize with the current `px4_msgs`:

| Topic | Symptom |
|-------|---------|
| `vehicle_local_position` | Fast CDR exception — field layout changed in 2025-02 |
| `failsafe_flags` | Fast CDR exception — field reordering in px4_msgs 2026-03 |
| `estimator_status` | rosbag player publish failure — type hash mismatch |

The remaining 8 checks work correctly from the bag. New bags recorded with the
current firmware will not have this issue.
