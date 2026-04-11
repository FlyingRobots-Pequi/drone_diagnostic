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
| `PX4: Vehicle Status` | `vehicle_status` | failsafe=False, arming ∈ {DISARMED, ARMED}, failure_detector=0x0 |
| `PX4: Failsafe Flags` | `failsafe_flags` | all 13 monitored flags False |
| `PX4: Land Detected` | `vehicle_land_detected` | freefall=False |
| `PX4: CPU Load` | `cpuload` | cpu ≤ 70 % AND ram ≤ 80 % |
| `PX4: Time Sync` | `timesync_status` | \|offset\| ≤ 500 µs |
| `PX4: Battery Health` | `battery_status` | PX4 warning=NONE, voltage ≥ 14.4 V, remaining ≥ 25 %, no faults, cell Δ ≤ 0.20 V |
| `PX4: IMU (sensor_combined)` | `sensor_combined` | rate ≥ 50 Hz |
| `PX4: Angular Velocity` | `vehicle_angular_velocity` | rate ≥ 50 Hz |
| `PX4: Local Position` | `vehicle_local_position` | xy_valid=True AND z_valid=True |
| `PX4: EKF Health` | `estimator_status` | topic alive, horiz_accuracy ≤ 5 m |
| `PX4: IMU Vibration` | `vehicle_imu_status` (primary) / `sensor_combined` (fallback) | accel_metric ≤ 0.35 m/s² AND gyro_metric ≤ 0.07 rad/s |
| `PX4: EKF Status Flags` | `estimator_status_flags` | all fs_* False, all reject_* False, tilt+yaw aligned |

### Status levels

| Level | Byte | Meaning |
|-------|------|---------|
| OK | `\x00` | Within all thresholds |
| WARN | `\x01` | Degraded but operational |
| ERROR | `\x02` | Out of bounds or topic absent |

Topics that have never been received return **WARN** for slow/status topics and **ERROR**
for high-rate topics (IMU, position, EKF).

## Configuration

All tunable parameters live in `config/diagnostics_config.yaml`. The file has two
independent sections — one for the aggregator (grouping) and one for the diagnostic
node (thresholds). Edit it in-repo and sync to the workspace before rebuilding:

```bash
cp config/diagnostics_config.yaml \
   ~/ros2_humble/src/drone_diagnostics/config/diagnostics_config.yaml
```

### Aggregator groups (`diagnostic_aggregator` section)

The `analyzers` block controls how `rqt_robot_monitor` groups checks. Each entry
creates one folder in the GUI.

```yaml
diagnostic_aggregator:
  ros__parameters:
    pub_rate: 1.0          # aggregator publish rate (Hz)
    analyzers:
      px4:                 # internal key — must be unique, no spaces
        type: diagnostic_aggregator/GenericAnalyzer
        path: PX4          # folder name shown in rqt_robot_monitor
        contains:
          - "PX4:"         # matches any check whose name contains this string
      my_group:            # add new groups the same way
        type: diagnostic_aggregator/GenericAnalyzer
        path: MyGroup
        contains:
          - "MyPrefix:"
```

The `contains` list is a substring match against the check name (the `name` field in
`/diagnostics`). Any check not matched by any group ends up under `Other`.

> **Important when running the aggregator manually** (not via the launch file):
> you must pass the node name explicitly so it matches the YAML key:
> ```bash
> ros2 run diagnostic_aggregator aggregator_node \
>     --ros-args -r __node:=diagnostic_aggregator \
>     --params-file config/diagnostics_config.yaml
> ```
> The launch file already sets this correctly.

### Node thresholds (`drone_diagnostic` section)

All thresholds that control OK/WARN/ERROR levels are here. No recompile needed —
just edit the YAML and relaunch.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_namespace` | `/pequi/hermit` | Namespace prefix for PX4 topics |
| `topic_timeout_sec` | `2.0` | Seconds until a topic is declared stale |
| `publish_rate_hz` | `2.0` | `/diagnostics` publish rate |
| `cpu_warn_pct` | `0.70` | CPU warn threshold (fraction 0–1) |
| `cpu_error_pct` | `0.90` | CPU error threshold (fraction 0–1) |
| `ram_warn_pct` | `0.80` | RAM error threshold (fraction 0–1) |
| `timesync_warn_us` | `500` | Clock offset warn (µs) |
| `timesync_error_us` | `2000` | Clock offset error (µs) |
| `battery_warn_v` | `14.4` | Battery voltage warn (V) — 4S LiPo default |
| `battery_error_v` | `13.6` | Battery voltage error (V) |
| `battery_warn_pct` | `0.25` | Remaining charge warn (fraction 0–1) |
| `battery_error_pct` | `0.10` | Remaining charge error (fraction 0–1) |
| `vibration_warn_m2s4` | `4.0` | Max accel variance warn (m²/s⁴) |
| `vibration_error_m2s4` | `9.0` | Max accel variance error (m²/s⁴) |
| `ekf_horiz_accuracy_warn_m` | `5.0` | EKF horizontal accuracy warn (m) |
| `rate_min_ratio` | `0.5` | Fraction of expected Hz below which a high-rate topic is WARN |

Example — tighten battery thresholds for a 6S LiPo:

```yaml
drone_diagnostic:
  ros__parameters:
    battery_warn_v: 21.6    # 6S: 3.6 V/cell × 6
    battery_error_v: 20.4   # 6S: 3.4 V/cell × 6
    battery_warn_pct: 0.20
    battery_error_pct: 0.05
```

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

# Terminal 1 — full pipeline (node + aggregator)
ros2 launch drone_diagnostics drone_diagnostics.launch.py

# Terminal 2 — play bag (--loop keeps topics alive indefinitely)
BAGS=/mnt/316f1a60-da34-40af-9076-4b3ee9fb0ed1/Documents/PMec/Flying/Diagnostic/rosbags
ros2 bag play $BAGS/bag_with_px4_60 --loop       # 60 s, loops forever
# ros2 bag play $BAGS/bag_with_px4 --loop        # 14 s, loops forever

# Terminal 3 — inspect
ros2 topic echo /diagnostics --once              # raw per-check output
ros2 topic echo /diagnostics_agg --once          # aggregated output

# Terminal 4 — GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

### What the `False` values mean in rqt_robot_monitor

When you expand a check in `rqt_robot_monitor` you will see key-value pairs with
`True`/`False`. This is normal — they are raw field values from the PX4 message,
not error indicators.

| Field | `False` means | `True` means |
|-------|--------------|--------------|
| `failsafe` | No failsafe active ✓ | Failsafe triggered ✗ |
| `freefall` | Not in freefall ✓ | Freefall detected ✗ |
| `landed` | In-air | On the ground |
| `xy_valid` | No horizontal fix | Position valid horizontally |
| `z_valid` | No altitude fix | Altitude valid |
| `xy_global` | No GPS global fix | GPS fix available |

With the test bags (`bag_with_px4`, `bag_with_px4_60`) the drone was sitting on the
ground, disarmed, with no GPS lock — so safety flags (`failsafe`, `freefall`) show
`False` (good), while position-validity flags (`z_valid`, `xy_global`) also show
`False` (expected without GPS outdoors).

### Expected status with `bag_with_px4_60`

| Check | Level | Reason |
|-------|-------|--------|
| Vehicle Status | OK | DISARMED \| AUTO_LOITER |
| Failsafe Flags | ERROR | `local_altitude_invalid`, `global_position_invalid`, `offboard_control_signal_lost` — drone on ground, no GPS |
| Land Detected | OK | LANDED |
| CPU Load | OK | ~41 % CPU, ~39 % RAM |
| Time Sync | ERROR | Drone clock not synced to Unix time (rosbag only) |
| Battery Health | WARN | Not recorded in bag |
| IMU | OK | ~100 Hz |
| Angular Velocity | OK | ~100 Hz |
| Local Position | WARN | `z_valid=False` — no barometer/GPS lock |
| EKF Health | OK | No innovation failures, accuracy 0.23 m |
| IMU Vibration | OK | ~0.00 m²/s⁴ (static drone) |
