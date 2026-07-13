#!/usr/bin/env bash
# 只开启错误退出、管道失败，关闭「未定义变量」检查
set -eo pipefail

ROOT_DIR="/workspace/RM"
MAIN_WS="${ROOT_DIR}/workspaces/rm_yolo_ros2_ws"
STEP_WS="${ROOT_DIR}/workspaces/step_motor_ros2_ws"

if [[ ! -d "${ROOT_DIR}" ]]; then
  echo "Error: ${ROOT_DIR} not found. Run this script inside the rm_humble container."
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Error: /opt/ros/humble/setup.bash not found. This container is not a ROS2 Humble environment."
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "==> Building main ROS2 workspace"
cd "${MAIN_WS}"
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
source install/setup.bash

echo "==> Building step motor ROS2 workspace"
cd "${STEP_WS}"
colcon build

cat <<'EOF'

Build finished.

Use these commands in new container terminals:

  source /opt/ros/humble/setup.bash
  source /workspace/RM/workspaces/rm_yolo_ros2_ws/install/setup.bash
  source /workspace/RM/workspaces/step_motor_ros2_ws/install/setup.bash

EOF

