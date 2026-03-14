# Drone Diagnostic Project

## System Overview
- **Drone**: PX4-based drone running ROS2 Humble (Ubuntu 22)
- **Flight software**: PX4 Autopilot via DDS/uXRCE bridge
- **VIO**: OpenVINS MSCKF (`ov_msckf`)
- **Drone namespace**: `/pequi/hermit`

## Prerequisites

```bash
sudo apt install ros-humble-diagnostic-updater \
                 ros-humble-diagnostic-aggregator \
                 ros-humble-rqt-robot-monitor
```

## Key Topics

### PX4 (BEST_EFFORT + TRANSIENT_LOCAL QoS)
> Note: uXRCE bridge uses TRANSIENT_LOCAL, not VOLATILE. DDS retains last value for late-joining subscribers. Use `--qos-reliability best_effort --qos-durability transient_local` when echoing manually.
| Topic | Type | Expected Hz |
|-------|------|-------------|
| `/pequi/hermit/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | ~10 |
| `/pequi/hermit/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | ~30 |
| `/pequi/hermit/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | ~10 |
| `/pequi/hermit/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | as needed |

### VIO / OpenVINS (RELIABLE + VOLATILE QoS)
| Topic | Type | Expected Hz |
|-------|------|-------------|
| `/ov_msckf/poseimu` | `geometry_msgs/PoseWithCovarianceStamped` | ~30 |
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | ~30 |

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

### Diagnostic Checks
| Name (in /diagnostics) | Category | What it detects |
|------------------------|----------|-----------------|
| `VIO: Pipeline Health` | VIO | VIO running but not reaching PX4 |
| `PX4: Offboard Control Mode` | PX4 | Rate + active mode flags |
| `PX4: Visual Odometry Bridge` | PX4 | Rate + position/velocity variance |
| `PX4: Trajectory Setpoint` | PX4 | Rate check |
| `VIO: OpenVINS Pose (poseimu)` | VIO | Rate + covariance diagonal |
| `VIO: OpenVINS Odometry (odomimu)` | VIO | Rate check |
| `System: ROS Log` | System | Error/warning counts from /rosout |

### Parameters
| Name | Default | Description |
|------|---------|-------------|
| `drone_namespace` | `/pequi/hermit` | ROS2 namespace prefix for PX4 topics |
| `topic_timeout_sec` | `2.0` | Seconds before a topic is declared stale |
| `publish_rate_hz` | `2.0` | Diagnostic publish rate |

### Thresholds
- `MAX_POSITION_VARIANCE = 0.25 m²` → warn if position std dev > 0.5m
- `MAX_VELOCITY_VARIANCE = 0.10 m²/s²`

## Diagnostic Pipeline

```
drone_diagnostic_node  →  /diagnostics  →  diagnostic_aggregator  →  /diagnostics_agg
                                                                             ↓
                                                                    rqt_robot_monitor
```

The aggregator groups statuses by name prefix into three categories:
- **PX4** — entries prefixed with `PX4:`
- **VIO** — entries prefixed with `VIO:`
- **System** — entries prefixed with `System:`

## Build & Run

```bash
# Copy package into your ROS2 workspace
cp -r /home/saraiva/Documents/PMec/Flying/Diagnostic/drone_diagnostics ~/ros2_ws/src/

# Build
cd ~/ros2_ws && colcon build --packages-select drone_diagnostics
source install/setup.bash

# Launch everything (diagnostic node + aggregator)
ros2 launch drone_diagnostics drone_diagnostics.launch.py

# Custom namespace example
ros2 launch drone_diagnostics drone_diagnostics.launch.py drone_namespace:=/other_drone

# Monitor
ros2 topic echo /diagnostics          # raw status
ros2 topic echo /diagnostics_agg      # aggregated by category
ros2 topic hz /diagnostics            # check publish rate

# GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
# or: rqt → Plugins → Robot Tools → Robot Monitor
```

## Validate Rosbag

```bash
# Info summary (check which topics have data)
ros2 bag info rosbag_logging_test-20260307T122527Z-1-001/rosbag_logging_test/

# Check if /diagnostics exists in bag
ros2 bag info <bag_path> | grep diagnostics

# Play bag + run diagnostics against it
ros2 bag play rosbag_logging_test-20260307T122527Z-1-001/rosbag_logging_test/ &
ros2 launch drone_diagnostics drone_diagnostics.launch.py
ros2 topic echo /diagnostics_agg --once

# Record only diagnostics from a live run
ros2 bag record /diagnostics /diagnostics_agg -o diagnostics_only
```

## Rosbag Notes
- **Recording from 2026-03-07** (`rosbag_logging_test-20260307T122527Z-1-001`)
- Format: ROS2 sqlite3, ~19 seconds, 233 messages
- **Known issue in bag**: All `/ov_msckf/*` topics have 0 messages — VIO was not running during that recording
- Only active topics in bag: `offboard_control_mode` (191 msgs) + `rosout` (42 msgs)
- The diagnostic node will report `ERROR: VIO pipeline completely offline` when played back
