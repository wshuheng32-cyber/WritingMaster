# 抓取 Demo 运行记录

本文件适用于交付目录 `RM_delivery`。交付目录通过 `docker-compose.yml`
挂载到容器内的路径是：

```bash
/workspace/RM
```

## 进入容器

在宿主机执行：

```bash
cd RM_delivery
xhost +local:
docker compose up -d
docker exec -it rm_humble bash
```

也可以直接使用脚本：

```bash
./scripts/enter_container.sh
```

## 首次编译

交付目录默认只带源码，不带 `build/`、`install/`、`log/`。

在容器内执行：

```bash
/workspace/RM/scripts/init_build.sh
```

## 启动抓取 Demo

### 终端 1：启动 RealSense 相机

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
```

### 终端 2：启动 robot_state_publisher（发布臂 TF 树）

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/rm_yolo_ros2_ws/install/setup.bash

ros2 launch rm_description rm_65_display.launch.py
```

### 终端 3：发布手眼标定 TF（eye-in-hand，Link6 → camera_link）

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/rm_yolo_ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.09110831 --y -0.03482909 --z 0.02311902 \
  --qx 0.00161397 --qy -0.00715724 --qz 0.70906855 --qw 0.70510138 \
  --frame-id Link6 \
  --child-frame-id camera_color_optical_frame
```

> 标定值来自 `tools/hand_eye_calibration/compute_in_hand.py`，数据集 `data20260611`。
> 重新标定后替换以上数值。
>
> **注意**：child-frame-id 必须是 `camera_color_optical_frame`（光学坐标系），
> 不能用 `camera_link`。原因：标定脚本使用 OpenCV `calibrateHandEye`，输出的是
> Link6 → 光学坐标系的变换；若发布为 camera_link，RealSense 驱动会再叠加一个
> camera_link → camera_color_optical_frame 旋转，导致坐标严重偏移。

### 终端 4：启动 RM 机械臂 driver

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/rm_yolo_ros2_ws/install/setup.bash

export LD_LIBRARY_PATH=/workspace/RM/workspaces/rm_yolo_ros2_ws/src/ros2_rm_robot/rm_driver/lib:/workspace/RM/workspaces/rm_yolo_ros2_ws/src/ros2_rm_robot/rm_driver/lib/linux_x86_c++_v1.1.3:$LD_LIBRARY_PATH

ros2 launch rm_driver rm_65_driver.launch.py
```

### 终端 5：启动步进电机串口节点

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/step_motor_ros2_ws/install/setup.bash

ros2 run step_motor motor_node --ros-args -p usart_port_name:=/dev/ttyACM0
```

### 终端 6：启动 YOLO 检测和抓取协调节点

确认模型文件路径存在。若模型文件放在主工作区根目录，使用下面命令：

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/step_motor_ros2_ws/install/setup.bash
source /workspace/RM/workspaces/rm_yolo_ros2_ws/install/setup.bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws

ros2 launch vi_grab vi_grab_demo.launch.py \
  model_path:=/workspace/RM/workspaces/rm_yolo_ros2_ws/yolov8n.pt \
  enable_motion:=true \
  gripper_backend:=step_motor
```

### 终端 7：发布要抓取或检测的目标名称

```bash
docker exec -it rm_humble bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/rm_yolo_ros2_ws/install/setup.bash

ros2 topic pub --once /choice_object std_msgs/msg/String "{data: 'bottle'}"
```

## 步进夹爪独立测试

张开：

```bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/step_motor_ros2_ws/install/setup.bash
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 200, dir: 0, mode: 2, angle: 30000, state: 0, sub_divide: 32}"
```

闭合：

```bash
source /opt/ros/humble/setup.bash
source /workspace/RM/workspaces/step_motor_ros2_ws/install/setup.bash
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 200, dir: 1, mode: 2, angle: 30000, state: 0, sub_divide: 32}"
```
