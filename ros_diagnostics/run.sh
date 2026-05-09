#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Load token
set -a && source "$SCRIPT_DIR/.env" && set +a

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_humble/install/setup.bash

# Copy latest node to workspace and rebuild
cp "$SCRIPT_DIR/drone_diagnostics/drone_diagnostics/diagnostic_node.py" \
   ~/ros2_humble/src/drone_diagnostics/drone_diagnostics/diagnostic_node.py
cp "$SCRIPT_DIR/drone_diagnostics/drone_diagnostics/diagnostics_logger_node.py" \
   ~/ros2_humble/src/drone_diagnostics/drone_diagnostics/diagnostics_logger_node.py

cd ~/ros2_humble
colcon build --packages-select drone_diagnostics --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Launch everything
ros2 launch drone_diagnostics drone_diagnostics.launch.py \
    enable_influxdb:=true \
    influxdb_url:=http://localhost:8086 \
    influxdb_org:=aerobatics \
    influxdb_bucket:=drone_diagnostics &

sleep 3

ros2 run rqt_robot_monitor rqt_robot_monitor &

wait
