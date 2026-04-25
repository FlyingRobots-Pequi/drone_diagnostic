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
│   ├── diagnostic_node.py       # 20 health checks → /diagnostics at 2 Hz
│   └── diagnostics_logger_node.py  # optional InfluxDB writer
├── config/
│   └── diagnostics_config.yaml  # aggregator groups + tunable thresholds + enable flags
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
| `PX4: GPS Quality` | `sensor_gps` | fix_type ≥ 3 (3D fix) AND hdop ≤ 2.0 AND satellites ≥ 6 AND jamming=0 |
| `PX4: RC Link` | `rc_channels` | signal_lost=False AND rssi ≥ 30 (if reported) |
| `PX4: Attitude` | `vehicle_attitude` | quaternion norm ≈ 1.0 AND no unexpected reset counter jumps |
| `PX4: ESC Health` | `esc_status` | no online ESC has failures AND temperature ≤ 70 °C |
| `PX4: Telemetry Link` | `telemetry_status` | at least one link heartbeat received |
| `PX4: Barometer` | `vehicle_air_data` | pressure within 30–110 kPa (plausibility check) |
| `PX4: Home Position` | `home_position` | at least one message received (home set) |
| `PX4: Geofence` | `geofence_result` | no active breach |

### Status levels

| Level | Byte | Meaning |
|-------|------|---------|
| OK | `\x00` | Within all thresholds |
| WARN | `\x01` | Degraded but operational |
| ERROR | `\x02` | Out of bounds or topic absent |

Topics that have never been received return **WARN** for slow/status topics and **ERROR**
for high-rate topics (IMU, position, EKF).

---

## Configuration

All tunable parameters live in `config/diagnostics_config.yaml`. No recompile is needed —
edit the YAML and relaunch. The file has two independent sections:

```bash
# After editing the YAML in this repo, sync to workspace:
cp config/diagnostics_config.yaml \
   ~/ros2_humble/src/drone_diagnostics/config/diagnostics_config.yaml
```

### Aggregator groups (`diagnostic_aggregator` section)

Controls how `rqt_robot_monitor` groups checks into folders. Each `analyzers` entry
creates one folder in the GUI using a substring match on the check name.

```yaml
diagnostic_aggregator:
  ros__parameters:
    pub_rate: 1.0
    analyzers:
      px4:                              # internal key — must be unique, no spaces
        type: diagnostic_aggregator/GenericAnalyzer
        path: PX4                       # folder name shown in rqt_robot_monitor
        contains:
          - "PX4:"                      # matches any check whose name contains this
      my_group:                         # add new groups the same way
        type: diagnostic_aggregator/GenericAnalyzer
        path: MyGroup
        contains:
          - "MyPrefix:"
```

Any check not matched by any group ends up under `Other`.

> **Running the aggregator manually** (not via the launch file): pass the node name
> explicitly so it matches the YAML key:
> ```bash
> ros2 run diagnostic_aggregator aggregator_node \
>     --ros-args -r __node:=diagnostic_aggregator \
>     --params-file config/diagnostics_config.yaml
> ```

### Enable / disable individual checks

Every check can be turned on or off independently without touching source code.
Set any `enable_*` flag to `false` in the YAML to suppress that check entirely.

```yaml
drone_diagnostic:
  ros__parameters:
    # ── Toggle individual checks ───────────────────────────────────────────────
    enable_vehicle_status:   true
    enable_failsafe:         true
    enable_land_detected:    true
    enable_cpuload:          true
    enable_timesync:         true
    enable_battery:          true
    enable_imu:              true
    enable_angular_velocity: true
    enable_local_position:   true
    enable_ekf_health:       true
    enable_imu_vibration:    true
    enable_ekf_status_flags: true
    enable_gps:              true
    enable_rc_link:          true
    enable_attitude:         true
    enable_esc_health:       true
    enable_telemetry_link:   true
    enable_barometer:        true
    enable_home_position:    true
    enable_geofence:         true
```

Example — disable GPS and RC link checks when flying indoors (no GPS, no RC):

```yaml
    enable_gps:     false
    enable_rc_link: false
```

### Node thresholds (`drone_diagnostic` section)

All thresholds that control OK/WARN/ERROR levels are here.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_namespace` | `/pequi/hermit` | Namespace prefix for PX4 topics |
| `topic_timeout_sec` | `2.0` | Seconds until a topic is declared stale |
| `publish_rate_hz` | `2.0` | `/diagnostics` publish rate |
| `cpu_warn_pct` | `0.70` | CPU warn threshold (fraction 0–1) |
| `cpu_error_pct` | `0.90` | CPU error threshold (fraction 0–1) |
| `ram_warn_pct` | `0.80` | RAM warn threshold (fraction 0–1) |
| `timesync_warn_us` | `500` | Clock offset warn (µs) |
| `timesync_error_us` | `2000` | Clock offset error (µs) |
| `battery_warn_v` | `14.4` | Battery voltage warn (V) — 4S LiPo default |
| `battery_error_v` | `13.6` | Battery voltage error (V) |
| `battery_warn_pct` | `0.25` | Remaining charge warn (fraction 0–1) |
| `battery_error_pct` | `0.10` | Remaining charge error (fraction 0–1) |
| `battery_cell_delta_warn_v` | `0.20` | Cell voltage imbalance warn (V) |
| `vibration_warn_m2s4` | `4.0` | Max accel variance warn (m²/s⁴) |
| `vibration_error_m2s4` | `9.0` | Max accel variance error (m²/s⁴) |
| `imu_accel_vib_warn_ms2` | `0.35` | Accel vibration metric warn (m/s²) |
| `imu_accel_vib_error_ms2` | `0.70` | Accel vibration metric error (m/s²) |
| `imu_gyro_vib_warn_rads` | `0.07` | Gyro vibration metric warn (rad/s) |
| `imu_gyro_vib_error_rads` | `0.15` | Gyro vibration metric error (rad/s) |
| `ekf_horiz_accuracy_warn_m` | `5.0` | EKF horizontal accuracy warn (m) |
| `rate_min_ratio` | `0.5` | Fraction of expected Hz below which a high-rate topic is WARN |
| `gps_fix_type_warn` | `3` | Warn if fix_type < 3 (no 3D fix) |
| `gps_fix_type_error` | `2` | Error if fix_type < 2 |
| `gps_hdop_warn` | `2.0` | Warn if HDOP > 2.0 |
| `gps_hdop_error` | `5.0` | Error if HDOP > 5.0 |
| `gps_satellites_warn` | `6` | Warn if satellites_used < 6 |
| `gps_satellites_error` | `4` | Error if satellites_used < 4 |
| `rc_rssi_warn` | `30` | Warn if RSSI < 30 (0–255 scale; 0 = not reported) |
| `rc_rssi_error` | `10` | Error if RSSI < 10 |
| `esc_temp_warn_c` | `70.0` | Warn if any ESC temperature > 70 °C |
| `esc_temp_error_c` | `85.0` | Error if any ESC temperature > 85 °C |
| `baro_pressure_min_pa` | `30000.0` | Error if pressure < 30 kPa (sensor frozen/stuck) |
| `baro_pressure_max_pa` | `110000.0` | Error if pressure > 110 kPa |

#### Common configuration examples

**6S LiPo battery:**
```yaml
drone_diagnostic:
  ros__parameters:
    battery_warn_v: 21.6     # 3.6 V/cell × 6
    battery_error_v: 20.4    # 3.4 V/cell × 6
    battery_warn_pct: 0.20
    battery_error_pct: 0.05
```

**Strict GPS for outdoor precision flight:**
```yaml
drone_diagnostic:
  ros__parameters:
    gps_fix_type_warn: 5     # require RTK float or better
    gps_hdop_warn: 1.2
    gps_satellites_warn: 10
```

**Indoor flight (disable GPS-dependent checks):**
```yaml
drone_diagnostic:
  ros__parameters:
    enable_gps:          false
    enable_rc_link:      false
    enable_home_position: false
```

---

## Adding a new diagnostic check

Every check follows the same five-step pattern. Adding one takes roughly 15 minutes
and touches four places in `diagnostic_node.py` plus the YAML config. The example
below adds a **Takeoff Status** check on `/fmu/out/takeoff_status`.

### Step 1 — Import the message type

At the top of `diagnostic_node.py`, add the new type to the `px4_msgs` import block:

```python
from px4_msgs.msg import (
    ...
    TakeoffStatus,   # ← add this
    ...
)
```

Find the available message types with:
```bash
ros2 interface list | grep px4_msgs
ros2 interface show px4_msgs/msg/TakeoffStatus   # inspect fields
```

### Step 2 — Declare parameters and state

Inside `DroneDiagnosticNode.__init__`, add the enable flag and any thresholds next
to the other declarations (order doesn't matter):

```python
# Enable flag — every check must have one
self.declare_parameter("enable_takeoff_status", True)

# Thresholds — only if your check has tunable values
# (skip this step if the check is purely boolean)
```

Then add a `None`-initialised message variable with the other state declarations:

```python
self._takeoff_status_msg: Optional[TakeoffStatus] = None
```

### Step 3 — Subscribe

In the subscriptions block, add one line:

```python
self.create_subscription(TakeoffStatus, f"{ns}/fmu/out/takeoff_status",
                         self._cb_takeoff_status, QOS_PX4)
```

And add the callback with the other `_cb_*` methods:

```python
def _cb_takeoff_status(self, msg: TakeoffStatus) -> None:
    self._takeoff_status_msg = msg
```

If the topic is **high-rate** (≥ 50 Hz), use a `TopicMonitor` instead of a bare
message variable — see `_cb_gps` / `_mon_gps` as an example.

### Step 4 — Write the diagnostic builder

Add a `_diag_*` method next to the other builders. The structure is always the same:
guard on `None` → read parameters → compute level → build KeyValue list → return status.

```python
def _diag_takeoff_status(self) -> DiagnosticStatus:
    # HEALTH CRITERIA — PX4: Takeoff Status (/fmu/out/takeoff_status)
    # ─────────────────────────────────────────────────────────────────
    # OK    : takeoff_state == RAMPUP (3) or FLIGHT (4) — motor ramp or flying
    # WARN  : takeoff_state == READY_FOR_TAKEOFF (2) — armed but not yet lifting
    # ERROR : takeoff_state == DISARMED (0) while we expected flight
    # NO MSG: WARN
    # ─────────────────────────────────────────────────────────────────
    TAKEOFF_STATE_NAMES = {0: "DISARMED", 1: "SPOOLUP", 2: "READY_FOR_TAKEOFF",
                           3: "RAMPUP", 4: "FLIGHT", 5: "LANDING"}

    if self._takeoff_status_msg is None:
        return DiagnosticStatus(level=WARN, name="PX4: Takeoff Status",
                                message="No messages received", hardware_id="px4_fmu")

    state = self._takeoff_status_msg.takeoff_state
    state_name = TAKEOFF_STATE_NAMES.get(state, f"UNKNOWN({state})")

    if state in (3, 4):       # RAMPUP or FLIGHT
        level = OK
    elif state == 2:           # READY_FOR_TAKEOFF
        level = WARN
    else:
        level = ERROR

    kvs = [KeyValue(key="takeoff_state", value=state_name)]
    return DiagnosticStatus(level=level, name="PX4: Takeoff Status",
                            message=state_name, hardware_id="px4_fmu", values=kvs)
```

**Rules to follow:**
- Name must match `"PX4: <Something>"` — the aggregator groups on this prefix.
- Always set `hardware_id="px4_fmu"`.
- Always include a `# HEALTH CRITERIA` block — it is the single source of truth for
  what the levels mean and keeps the code self-documenting.
- No message received → return **WARN** for slow/status topics, **ERROR** for
  high-rate topics the drone always publishes (IMU, position).
- Read thresholds via `self.get_parameter(...)` inside the method, not in `__init__`,
  so they can be changed at runtime with `ros2 param set`.

### Step 5 — Wire into the publish loop and YAML

In `_publish_diagnostics`, add one line in the same style as the others:

```python
if self._bp("enable_takeoff_status"): statuses.append(self._diag_takeoff_status())
```

In `config/diagnostics_config.yaml`, add the enable flag and any thresholds under
`drone_diagnostic: ros__parameters:`:

```yaml
drone_diagnostic:
  ros__parameters:
    enable_takeoff_status: true   # set false to suppress this check entirely
```

### Checklist

- [ ] Message type imported in `px4_msgs` block
- [ ] `enable_<name>` parameter declared in `__init__`
- [ ] Threshold parameters declared in `__init__` (if any)
- [ ] `self._<name>_msg` state variable initialised to `None`
- [ ] `create_subscription` added with `QOS_PX4`
- [ ] `_cb_<name>` callback defined
- [ ] `_diag_<name>` method with `# HEALTH CRITERIA` block
- [ ] One line added to `_publish_diagnostics` behind the enable flag
- [ ] `enable_<name>: true` added to `diagnostics_config.yaml`
- [ ] Sync YAML and `.py` to `~/ros2_humble/src/drone_diagnostics/` and rebuild

### Verifying your new check

```bash
# Rebuild
cd ~/ros2_humble && colcon build --packages-select drone_diagnostics && source install/setup.bash

# Run node + bag
ros2 run drone_diagnostics diagnostic_node &
ros2 bag play rosbags/bag-with-sensors --loop --clock &

# Confirm your check appears
ros2 topic echo /diagnostics --once | grep -A5 "Takeoff Status"

# Disable it at runtime (no restart needed)
ros2 param set /drone_diagnostic enable_takeoff_status false
```

---

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

## Docker / InfluxDB

`docker-compose.yml` starts an InfluxDB 2.7 instance alongside the ROS2 node.
All tunables live in `.env`.

```bash
# Start InfluxDB (and optionally the diagnostic node)
docker compose up -d influxdb

# Tear down and delete all stored data
docker compose down -v
```

### Storage configuration

InfluxDB 2.x OSS has no hard byte-size disk cap. Storage is managed via two
mechanisms, both configurable in `.env`:

| Variable | Default | Effect |
|---|---|---|
| `INFLUXDB_RETENTION` | `90d` | Bucket retention period — data older than this is deleted automatically |
| `INFLUXDB_CACHE_MAX_MEMORY_SIZE` | `104857600` (100 MB) | In-memory write-cache cap (bytes) |

The retention period is the primary knob. At this project's write rate (~2 Hz,
small payloads), `90d` keeps total storage well under 1 GB.

```bash
INFLUXDB_RETENTION=7d    # ~1 week of history
INFLUXDB_RETENTION=365d  # 1 year (still < 1 GB for this workload)
INFLUXDB_RETENTION=0     # infinite retention — manage storage yourself
```

> **Hard disk quota**: if you need a true byte-size limit, apply an OS-level
> filesystem quota to the Docker volume host path
> (`/var/lib/docker/volumes/diagnostic_influxdb_data`).

---

## Monitor

```bash
# Raw status array (one entry per check)
ros2 topic echo /diagnostics

# Aggregated by group (PX4)
ros2 topic echo /diagnostics_agg

# GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

## Test with rosbag

```bash
cd ~/ros2_humble && source install/setup.bash

# Terminal 1 — full pipeline (node + aggregator)
ros2 launch drone_diagnostics drone_diagnostics.launch.py

# Terminal 2 — play bag (--loop keeps topics alive indefinitely)
BAGS=/mnt/316f1a60-da34-40af-9076-4b3ee9fb0ed1/Documents/PMec/Flying/Diagnostic/rosbags
ros2 bag play $BAGS/bag-with-sensors --loop --clock   # 73 s, in-flight data, recommended
# ros2 bag play $BAGS/bag_with_px4_60 --loop --clock  # 60 s, ground/static data

# Terminal 3 — inspect
ros2 topic echo /diagnostics --once     # raw per-check output
ros2 topic echo /diagnostics_agg --once # aggregated output

# Terminal 4 — GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

### Available bags

| Bag | Duration | Drone state | Notable topics |
|-----|----------|-------------|----------------|
| `bag-with-sensors` | 73 s | **In-flight** | battery_status, vehicle_attitude, estimator_status_flags, LiDAR scan, RealSense images |
| `bag_with_px4_60` | 60 s | Ground / static | Core fmu/out topics only (no battery, no sensors) |
| `bag_with_px4` | 14 s | Ground / static | Same as above, shorter |
| `rosbag_logging_test` | 19 s | No fmu/out data | Only offboard_control_mode + rosout |

### Expected status with `bag-with-sensors` (recommended)

| Check | Level | Notes |
|-------|-------|-------|
| `PX4: Vehicle Status` | WARN | GCS link lost; `pre_flight_checks_pass=False` |
| `PX4: Failsafe Flags` | ERROR | `global_position_invalid` — no GPS in this flight |
| `PX4: Land Detected` | OK | IN-AIR |
| `PX4: CPU Load` | OK | ~43–49 % CPU, ~44 % RAM |
| `PX4: Time Sync` | ERROR | Drone clock not synced to Unix time (known rosbag issue) |
| `PX4: Battery Health` | OK | ~15.6 V, ~55 %, ~1.7–3.9 A |
| `PX4: IMU (sensor_combined)` | OK | ~97–108 Hz |
| `PX4: Angular Velocity` | OK | ~96–108 Hz |
| `PX4: Local Position` | OK | ~97–100 Hz, `xy_valid=True`, `z_valid=True` |
| `PX4: EKF Health` | OK | ~92–99 Hz, horiz accuracy ~1 cm |
| `PX4: IMU Vibration` | OK | ~0.00–0.01 m²/s⁴ |
| `PX4: EKF Status Flags` | WARN | `yaw_not_aligned` — no magnetometer/GPS fusion |
| `PX4: GPS Quality` | WARN | `sensor_gps` not recorded in this bag |
| `PX4: RC Link` | WARN | `rc_channels` not recorded in this bag |
| `PX4: Attitude` | OK | Quaternion norm=1.0, 4 resets (normal startup) |
| `PX4: ESC Health` | WARN | `esc_status` not recorded in this bag |
| `PX4: Telemetry Link` | WARN | `telemetry_status` not recorded in this bag |
| `PX4: Barometer` | WARN | `vehicle_air_data` not recorded in this bag |
| `PX4: Home Position` | WARN | Topic present but 0 messages recorded |
| `PX4: Geofence` | OK | No geofence configured |

### Expected status with `bag_with_px4_60` (ground/static)

| Check | Level | Notes |
|-------|-------|-------|
| `PX4: Vehicle Status` | OK | DISARMED \| AUTO_LOITER |
| `PX4: Failsafe Flags` | ERROR | `local_altitude_invalid`, `global_position_invalid`, `offboard_control_signal_lost` |
| `PX4: Land Detected` | OK | LANDED |
| `PX4: CPU Load` | OK | ~41 % CPU, ~39 % RAM |
| `PX4: Time Sync` | ERROR | Drone clock not synced to Unix time |
| `PX4: Battery Health` | WARN | Not recorded in bag |
| `PX4: IMU (sensor_combined)` | OK | ~100 Hz |
| `PX4: Angular Velocity` | OK | ~100 Hz |
| `PX4: Local Position` | OK | topic alive (CDR mismatch resolved in newer bags) |
| `PX4: EKF Health` | OK | No innovation failures |
| `PX4: IMU Vibration` | OK | ~0.00 m²/s⁴ (static drone) |
| `PX4: EKF Status Flags` | WARN | Likely stale — not all flags available in older bag |
| `PX4: GPS Quality` | WARN | Not in bag |
| `PX4: RC Link` | WARN | Not in bag |
| `PX4: Attitude` | OK | Quaternion norm=1.0 |
| `PX4: ESC Health` | WARN | Not in bag |
| `PX4: Telemetry Link` | WARN | Not in bag |
| `PX4: Barometer` | WARN | Not in bag |
| `PX4: Home Position` | WARN | Not in bag |
| `PX4: Geofence` | OK | No geofence configured |

### What the `True`/`False` values mean in rqt_robot_monitor

When you expand a check you will see key-value pairs. These are raw PX4 field values,
not error indicators.

| Field | `False` means | `True` means |
|-------|--------------|--------------|
| `failsafe` | No failsafe active ✓ | Failsafe triggered ✗ |
| `freefall` | Not in freefall ✓ | Freefall detected ✗ |
| `landed` | In-air | On the ground |
| `xy_valid` | No horizontal fix | Position valid horizontally |
| `z_valid` | No altitude fix | Altitude valid |
| `xy_global` | No GPS global fix | GPS fix available |
