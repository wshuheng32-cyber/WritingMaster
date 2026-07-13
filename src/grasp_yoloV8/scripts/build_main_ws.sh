#!/usr/bin/env bash
set -euo pipefail

cd /workspace/RM/workspaces/rm_yolo_ros2_ws
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
