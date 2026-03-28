# Drone Diagnostic Project

## System Overview
- **Drone**: PX4-based drone running ROS2 Humble (Ubuntu 22)
- **Flight software**: PX4 Autopilot via DDS/uXRCE bridge
- **VIO**: OpenVINS MSCKF (`ov_msckf`) — not monitored by diagnostic node yet
- **Drone namespace**: `/pequi/hermit`
- **ROS2 workspace**: `/home/saraiva/ros2_humble`

## Prerequisites

```bash
sudo apt install ros-humble-diagnostic-updater \
                 ros-humble-diagnostic-aggregator \
                 ros-humble-rqt-robot-monitor
```

## Key Topics Monitored

### fmu/out — telemetry from PX4 (BEST_EFFORT + TRANSIENT_LOCAL QoS)
> Note: uXRCE bridge uses TRANSIENT_LOCAL, not VOLATILE. DDS retains last value for late-joining
> subscribers. Use `--qos-reliability best_effort --qos-durability transient_local` when echoing manually.

| Topic | Type | Expected Hz |
|-------|------|-------------|
| `/pequi/hermit/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | ~2 |
| `/pequi/hermit/fmu/out/failsafe_flags` | `px4_msgs/FailsafeFlags` | ~2 |
| `/pequi/hermit/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | ~1 |
| `/pequi/hermit/fmu/out/cpuload` | `px4_msgs/Cpuload` | ~2 |
| `/pequi/hermit/fmu/out/timesync_status` | `px4_msgs/TimesyncStatus` | ~1 |
| `/pequi/hermit/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | ~1 |
| `/pequi/hermit/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | ~100 |
| `/pequi/hermit/fmu/out/vehicle_angular_velocity` | `px4_msgs/VehicleAngularVelocity` | ~100 |
| `/pequi/hermit/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | ~100 |
| `/pequi/hermit/fmu/out/estimator_status` | `px4_msgs/EstimatorStatus` | ~100 |

## Package Structure

```
drone_diagnostics/
├── drone_diagnostics/
│   └── diagnostic_node.py   # main node — publishes /diagnostics
├── config/
│   └── diagnostics_config.yaml  # aggregator grouping config
├── launch/
│   └── drone_diagnostics.launch.py  # starts node + aggregator
├── package.xml
├── setup.py
└── setup.cfg
```

## Diagnostic Node

**Publishes**: `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) at 2 Hz

### Diagnostic Checks (11 total — all /fmu/out)

Each function has an explicit `HEALTH CRITERIA` block in the source.

| Name (in /diagnostics) | Source Topic | OK condition |
|------------------------|--------------|--------------|
| `PX4: Vehicle Status` | `vehicle_status` | failsafe=False, arming ∈ {DISARMED, ARMED} |
| `PX4: Failsafe Flags` | `failsafe_flags` | all 13 monitored flags False |
| `PX4: Land Detected` | `vehicle_land_detected` | freefall=False |
| `PX4: CPU Load` | `cpuload` | cpu ≤ 70% AND ram ≤ 80% |
| `PX4: Time Sync` | `timesync_status` | \|offset\| ≤ 500 µs |
| `PX4: Battery Health` | `battery_status` | voltage ≥ 14.4V AND remaining ≥ 25% |
| `PX4: IMU (sensor_combined)` | `sensor_combined` | rate ≥ 50 Hz |
| `PX4: Angular Velocity` | `vehicle_angular_velocity` | rate ≥ 50 Hz |
| `PX4: Local Position` | `vehicle_local_position` | xy_valid=True AND z_valid=True |
| `PX4: EKF Health` | `estimator_status` | no innovation failures, horiz_accuracy ≤ 5m |
| `PX4: IMU Vibration` | `sensor_combined` (derived) | max accel variance ≤ 4.0 m²/s⁴ |

### Parameters
| Name | Default | Description |
|------|---------|-------------|
| `drone_namespace` | `/pequi/hermit` | ROS2 namespace prefix for PX4 topics |
| `topic_timeout_sec` | `2.0` | Seconds before a topic is declared stale |
| `publish_rate_hz` | `2.0` | Diagnostic publish rate |
| `cpu_warn_pct` | `0.70` | CPU warn threshold |
| `cpu_error_pct` | `0.90` | CPU error threshold |
| `ram_warn_pct` | `0.80` | RAM error threshold |
| `timesync_warn_us` | `500` | Clock offset warn (µs) |
| `timesync_error_us` | `2000` | Clock offset error (µs) |

## Diagnostic Pipeline

```
drone_diagnostic_node  →  /diagnostics  →  diagnostic_aggregator  →  /diagnostics_agg
                                                                             ↓
                                                                    rqt_robot_monitor
```

The aggregator groups statuses by name prefix:
- **PX4** — entries prefixed with `PX4:`

## Build & Run

```bash
# Workspace is at ~/ros2_humble (not ~/ros2_ws)
# Package is already in ~/ros2_humble/src/drone_diagnostics
# After editing, sync from repo:
cp .../Diagnostic/drone_diagnostics/drone_diagnostics/diagnostic_node.py \
   ~/ros2_humble/src/drone_diagnostics/drone_diagnostics/

# Build
cd ~/ros2_humble && source install/setup.bash
colcon build --packages-select drone_diagnostics
source install/setup.bash

# Run node only
ros2 run drone_diagnostics diagnostic_node

# Launch everything (diagnostic node + aggregator)
ros2 launch drone_diagnostics drone_diagnostics.launch.py

# Monitor
ros2 topic echo /diagnostics          # raw status
ros2 topic echo /diagnostics_agg      # aggregated by category

# GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## Rosbags

All bags are under: `/mnt/316f1a60-da34-40af-9076-4b3ee9fb0ed1/Documents/PMec/Flying/Diagnostic/rosbags/`

### bag_with_px4
- **Path**: `rosbags/bag_with_px4`
- **Duration**: 14s | **Messages**: 9021 | **Date**: 2026-03-07
- **fmu/out topics with data**: sensor_combined (1422), vehicle_angular_velocity (1422),
  vehicle_local_position (1425), estimator_status (1425), vehicle_status (28),
  timesync_status (15), cpuload (29), failsafe_flags (36), vehicle_land_detected (15)
- **Missing**: battery_status (not recorded)
- **Known issue**: `estimator_status`, `vehicle_local_position`, `failsafe_flags` fail CDR
  deserialization — px4_msgs version mismatch between workspace and bag recording

### bag_with_px4_60
- **Path**: `rosbags/bag_with_px4_60`
- **Duration**: 60s | **Messages**: 38549 | **Date**: 2026-03-07
- **fmu/out topics with data**: same as bag_with_px4 but ~4× more messages
- **Same CDR issue** as bag_with_px4

### rosbag_logging_test
- **Path**: `rosbags/rosbag_logging_test`
- **Duration**: 19s | **Messages**: 233 | **Date**: 2026-03-03
- **Active topics**: `/fmu/in/offboard_control_mode` (191 msgs), `/rosout` (42 msgs)
- **No fmu/out data** — all diagnostic checks will report ERROR/WARN (expected)

### Test with a bag

```bash
cd ~/ros2_humble && source install/setup.bash
ros2 run drone_diagnostics diagnostic_node &
sleep 2
ros2 bag play rosbags/bag_with_px4 --clock &
sleep 5
ros2 topic echo /diagnostics --once
```

### Expected results with bag_with_px4

| Check | Expected | Actual (tested 2026-03-20) |
|-------|----------|---------------------------|
| Vehicle Status | OK | OK — DISARMED \| AUTO_LOITER |
| Failsafe Flags | OK | WARN — CDR mismatch (msg version) |
| Land Detected | OK | OK — LANDED |
| CPU Load | OK | OK — CPU 41% RAM 39% |
| Time Sync | ERROR | ERROR — drone clock not synced with unix time |
| Battery Health | WARN | WARN — not in bag |
| IMU | OK | OK — 100.2 Hz |
| Angular Velocity | OK | OK — 100.3 Hz |
| Local Position | OK | ERROR — CDR mismatch (msg version) |
| EKF Health | OK | ERROR — CDR mismatch (msg version) |
| IMU Vibration | OK | OK — 0.00 m²/s⁴ (static drone) |

### CDR Deserialization Issue

`estimator_status`, `vehicle_local_position`, and `failsafe_flags` fail to deserialize
from the bags with:
```
Fast CDR exception deserializing message of type px4_msgs::msg::dds_::EstimatorStatus_
```
Root cause: px4_msgs in the workspace was compiled from a different PX4 version than
what generated the bag. To fix: record new bags after re-building px4_msgs from the
exact PX4 version running on the drone.
