````md
# Codex 任务：将 RealMan YOLOv8 视觉抓取项目从 ROS1 重构为 ROS2 Humble

你现在在一个 Docker 容器中工作，环境如下：

- 宿主机：Ubuntu 24.04
- Docker 镜像：`osrf/ros:humble-desktop`
- ROS 版本：ROS2 Humble
- 工作目录：`/workspace/RM`
- ROS2 工作空间：`/workspace/RM/workspaces/rm_yolo_ros2_ws`
- ROS2 src 目录：`/workspace/RM/workspaces/rm_yolo_ros2_ws/src`
- 已 clone 并编译通过的 RM 官方 ROS2 包：
  - `/workspace/RM/workspaces/rm_yolo_ros2_ws/src/ros2_rm_robot`
- 原始 ROS1 YOLOv8 项目，仅作为迁移参考：
  - `/workspace/RM/references/original_yolov8_ros1`
- 原始项目仓库：
  - `https://github.com/RealManRobot/YOLOv8-Visual-Recognition`
- RM 官方 ROS2 仓库：
  - `https://github.com/RealManRobot/ros2_rm_robot`

背景信息：
- 原始 YOLOv8 项目是 ROS1 Noetic 项目，使用 `catkin build`、`roslaunch`、`rostopic`，其中主要包含 `vi_grab`、`vi_msgs`、`rm_robot`。
- 原项目的关键脚本是：
  - `/workspace/RM/references/original_yolov8_ros1/vi_grab/scripts/vi_catch_yolov8.py`
  - `/workspace/RM/references/original_yolov8_ros1/vi_grab/scripts/vision_grab.py`
- 原项目的自定义消息是：
  - `/workspace/RM/references/original_yolov8_ros1/vi_msgs/msg/ObjectInfo.msg`
- 目标是不要把 ROS1 包直接放进 ROS2 workspace 编译，而是在 ROS2 工作空间中创建新的 ROS2 包：
  - `vi_msgs`
  - `vi_grab`
- RM 官方 ROS2 包 `ros2_rm_robot` 面向 Ubuntu 22.04 + ROS2 Humble，支持 RM65，并提供 `rm_ros_interfaces`、`rm_driver`、`rm_bringup` 等包。RM 官方文档和 README 均说明 ROS2 版本为 Humble，支持 RM65 系列。请基于这个 ROS2 包做机械臂控制，不要重写底层驱动。
- 原 YOLOv8 项目是 ROS1 节点，用于发布 YOLOv8 视觉识别结果，并订阅目标类别完成视觉抓取；这个逻辑需要迁移到 ROS2。

请完成以下任务。

---

## 任务 1：检查当前工作空间

在开始修改前，先检查：

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws
ls src
````

确认至少存在：

```text
ros2_rm_robot
```

如果 `vi_msgs` 和 `vi_grab` 不存在，则创建它们。

---

## 任务 2：创建 ROS2 消息包 `vi_msgs`

如果 `/workspace/RM/workspaces/rm_yolo_ros2_ws/src/vi_msgs` 不存在，执行：

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws/src

ros2 pkg create vi_msgs \
  --build-type ament_cmake \
  --dependencies std_msgs
```

然后创建消息文件：

路径：

```text
/workspace/RM/workspaces/rm_yolo_ros2_ws/src/vi_msgs/msg/ObjectInfo.msg
```

内容：

```text
std_msgs/Header header
string object_class
float64 x
float64 y
float64 z
float64 score
int32 center_x
int32 center_y
```

重写 `vi_msgs/CMakeLists.txt` 为：

```cmake
cmake_minimum_required(VERSION 3.8)
project(vi_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectInfo.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

修改 `vi_msgs/package.xml`，确保包含：

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<build_depend>std_msgs</build_depend>
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>std_msgs</exec_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## 任务 3：创建 ROS2 Python 包 `vi_grab`

如果 `/workspace/RM/workspaces/rm_yolo_ros2_ws/src/vi_grab` 不存在，执行：

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws/src

ros2 pkg create vi_grab \
  --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge message_filters tf2_ros tf2_geometry_msgs vi_msgs rm_ros_interfaces
```

包结构需要整理成：

```text
vi_grab
├── package.xml
├── setup.py
├── resource
│   └── vi_grab
├── vi_grab
│   ├── __init__.py
│   ├── object_detector_node.py
│   └── grasp_coordinator_node.py
└── launch
    └── vi_grab_demo.launch.py
```

如没有 `launch` 目录，请创建。

---

## 任务 4：实现 `object_detector_node.py`

创建或重写：

```text
/workspace/RM/workspaces/rm_yolo_ros2_ws/src/vi_grab/vi_grab/object_detector_node.py
```

目标：

* 用 ROS2 `rclpy` 实现节点 `object_detect`
* 订阅 RealSense ROS2 wrapper 的彩色图、对齐深度图和相机内参
* 使用 Ultralytics YOLOv8 检测目标
* 将检测框中心点通过深度图反投影成相机坐标系下的 3D 点
* 发布 `/object_pose`，消息类型为 `vi_msgs/msg/ObjectInfo`
* 保留 OpenCV 可视化窗口，参数控制是否显示

要求：

* 默认模型：`yolov8n.pt`
* 默认置信度：`0.5`
* 默认订阅话题：

  * `/camera/camera/color/image_raw`
  * `/camera/camera/aligned_depth_to_color/image_raw`
  * `/camera/camera/color/camera_info`
* 发布话题：

  * `/object_pose`
* 深度处理：

  * 如果深度图是 `uint16`，单位按 mm 转 m
  * 如果深度图是 `float32`，默认单位为 m
  * 对中心点附近 5x5 区域取有效深度中位数，避免深度空洞
* 消息字段：

  * `header` 使用彩色图 header
  * `header.frame_id` 使用图像 frame_id，如果为空则使用 `camera_color_optical_frame`
  * `object_class`
  * `x y z`
  * `score`
  * `center_x center_y`

请使用以下实现作为基础，可根据实际 lint 修正：

```python
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vi_msgs.msg import ObjectInfo
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detect')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('show_image', True)

        model_path = self.get_parameter('model_path').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.show_image = bool(self.get_parameter('show_image').value)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.pub = self.create_publisher(ObjectInfo, '/object_pose', 10)

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self.camera_info_callback,
            10
        )

        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.08
        )
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('YOLOv8 object detector started.')

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_to_meter(self, depth_img, u, v):
        h, w = depth_img.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        x1, x2 = max(0, u - 2), min(w, u + 3)
        y1, y2 = max(0, v - 2), min(h, v + 3)
        patch = depth_img[y1:y2, x1:x2].astype(np.float32)
        valid = patch[patch > 0]

        if valid.size == 0:
            return None

        z = float(np.median(valid))

        if depth_img.dtype == np.uint16:
            z *= 0.001

        if z <= 0.0 or np.isnan(z):
            return None

        return z

    def deproject(self, u, v, z):
        x = (u - self.cx) / self.fx * z
        y = (v - self.cy) / self.fy * z
        return x, y, z

    def image_callback(self, color_msg: Image, depth_msg: Image):
        if self.fx is None:
            return

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        result = self.model.predict(color, conf=self.confidence, verbose=False)[0]
        canvas = result.plot()

        if result.boxes is None:
            return

        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        cls_ids = result.boxes.cls.cpu().numpy().astype(int)

        for box, score, cls_id in zip(boxes, confs, cls_ids):
            x1, y1, x2, y2 = box.astype(int)
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            z = self.depth_to_meter(depth, u, v)
            if z is None:
                continue

            x, y, z = self.deproject(u, v, z)
            name = result.names[int(cls_id)]

            msg = ObjectInfo()
            msg.header = color_msg.header
            if not msg.header.frame_id:
                msg.header.frame_id = 'camera_color_optical_frame'
            msg.object_class = str(name)
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            msg.score = float(score)
            msg.center_x = int(u)
            msg.center_y = int(v)
            self.pub.publish(msg)

            cv2.circle(canvas, (u, v), 4, (255, 255, 255), -1)
            cv2.putText(
                canvas,
                f'{name} ({x:.3f},{y:.3f},{z:.3f})',
                (u + 10, v + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        if self.show_image:
            cv2.imshow('detection', canvas)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 任务 5：实现 `grasp_coordinator_node.py`

创建或重写：

```text
/workspace/RM/workspaces/rm_yolo_ros2_ws/src/vi_grab/vi_grab/grasp_coordinator_node.py
```

目标：

* ROS2 节点名：`object_catch`
* 订阅：

  * `/choice_object`，类型 `std_msgs/msg/String`
  * `/object_pose`，类型 `vi_msgs/msg/ObjectInfo`
  * `/rm_driver/get_current_arm_state_result`，类型来自 `rm_ros_interfaces/msg/Armstate`
* 定时发布：

  * `/rm_driver/get_current_arm_state_cmd`，类型 `std_msgs/msg/Empty`
* 控制话题：

  * `/rm_driver/movej_p_cmd`
  * `/rm_driver/movel_cmd`
  * `/rm_driver/set_gripper_pick_cmd`
* 使用 TF2 将物体点从相机坐标系转换到机械臂基坐标系
* 参数：

  * `base_frame`，默认 `base_link`
  * `approach_offset`，默认 `0.07`
  * `lift_height`，默认 `0.05`
  * `speed`，默认 `20`
  * `enable_motion`，默认 `false`
* `enable_motion=false` 时只打印目标点，不控制真实机械臂
* `enable_motion=true` 时才执行：

  1. movej_p 到目标前方
  2. movel 到目标点
  3. 闭合夹爪
  4. movel 抬起

注意：

* 不要在 ROS2 subscriber 回调里长时间阻塞。
* 可以用线程执行抓取流程。
* 保持当前机械臂末端姿态抓取。
* 需要根据 `rm_ros_interfaces` 中真实消息字段修正 `Movejp`、`Movel`、`Gripperpick` 的字段名。如果字段不一致，请用以下命令查看真实接口：

  * `ros2 interface show rm_ros_interfaces/msg/Movejp`
  * `ros2 interface show rm_ros_interfaces/msg/Movel`
  * `ros2 interface show rm_ros_interfaces/msg/Gripperpick`
  * `ros2 interface show rm_ros_interfaces/msg/Armstate`

请先根据接口实际字段适配，不要臆造不存在的字段。

实现时可以参考以下骨架：

```python
import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PointStamped, Pose

from vi_msgs.msg import ObjectInfo

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from rm_ros_interfaces.msg import Armstate, Movejp, Movel, Gripperpick


class GraspCoordinatorNode(Node):
    def __init__(self):
        super().__init__('object_catch')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('approach_offset', 0.07)
        self.declare_parameter('lift_height', 0.05)
        self.declare_parameter('speed', 20)
        self.declare_parameter('enable_motion', False)

        self.base_frame = self.get_parameter('base_frame').value
        self.approach_offset = float(self.get_parameter('approach_offset').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.speed = int(self.get_parameter('speed').value)
        self.enable_motion = bool(self.get_parameter('enable_motion').value)

        self.target_class = ''
        self.latest_arm_state = None
        self.busy = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(String, '/choice_object', self.choice_callback, 10)
        self.create_subscription(ObjectInfo, '/object_pose', self.object_callback, 10)
        self.create_subscription(Armstate, '/rm_driver/get_current_arm_state_result', self.arm_state_callback, 10)

        self.arm_state_req_pub = self.create_publisher(Empty, '/rm_driver/get_current_arm_state_cmd', 10)
        self.movejp_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.movel_pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.gripper_pub = self.create_publisher(Gripperpick, '/rm_driver/set_gripper_pick_cmd', 10)

        self.create_timer(0.2, self.request_arm_state)

        self.get_logger().warn(
            f'Grasp coordinator started. enable_motion={self.enable_motion}. '
            '调试确认无误后再打开真实运动。'
        )

    def request_arm_state(self):
        self.arm_state_req_pub.publish(Empty())

    def arm_state_callback(self, msg: Armstate):
        self.latest_arm_state = msg

    def choice_callback(self, msg: String):
        self.target_class = msg.data.strip()
        self.busy = False
        self.get_logger().info(f'收到抓取目标: {self.target_class}')

    def object_callback(self, msg: ObjectInfo):
        if self.busy:
            return

        if not self.target_class:
            return

        if msg.object_class != self.target_class:
            return

        if self.latest_arm_state is None:
            self.get_logger().warn('还没有收到机械臂状态，等待 /rm_driver/get_current_arm_state_result')
            return

        try:
            base_point = self.transform_object_to_base(msg)
        except Exception as e:
            self.get_logger().warn(f'TF 转换失败: {e}')
            return

        self.busy = True
        self.target_class = ''

        t = threading.Thread(
            target=self.execute_grasp,
            args=(base_point, self.latest_arm_state),
            daemon=True
        )
        t.start()

    def transform_object_to_base(self, obj: ObjectInfo):
        point = PointStamped()
        point.header = obj.header
        point.point.x = obj.x
        point.point.y = obj.y
        point.point.z = obj.z

        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            point.header.frame_id,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.3)
        )

        return do_transform_point(point, transform)

    def make_pose(self, x, y, z, arm_state: Armstate):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        # 根据 Armstate 的实际字段调整。
        # 目标是保持当前末端姿态。
        pose.orientation = arm_state.pose.orientation
        return pose

    def publish_movejp(self, pose: Pose):
        msg = Movejp()
        # TODO: 根据 ros2 interface show rm_ros_interfaces/msg/Movejp 的实际字段修正
        msg.pose = pose
        msg.speed = self.speed
        msg.trajectory_connect = 0
        msg.block = True
        self.movejp_pub.publish(msg)

    def publish_movel(self, pose: Pose):
        msg = Movel()
        # TODO: 根据 ros2 interface show rm_ros_interfaces/msg/Movel 的实际字段修正
        msg.pose = pose
        msg.speed = self.speed
        msg.trajectory_connect = 0
        msg.block = True
        self.movel_pub.publish(msg)

    def close_gripper(self):
        msg = Gripperpick()
        # TODO: 根据 ros2 interface show rm_ros_interfaces/msg/Gripperpick 的实际字段修正
        msg.speed = 200
        msg.force = 200
        msg.block = True
        msg.timeout = 10
        self.gripper_pub.publish(msg)

    def execute_grasp(self, base_point: PointStamped, arm_state: Armstate):
        x = base_point.point.x
        y = base_point.point.y
        z = base_point.point.z

        self.get_logger().info(
            f'目标 {self.base_frame} 坐标: x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

        if not self.enable_motion:
            self.get_logger().warn('enable_motion=false，仅打印目标坐标，不执行真实抓取。')
            self.busy = False
            return

        approach_pose = self.make_pose(x - self.approach_offset, y, z, arm_state)
        self.publish_movejp(approach_pose)
        self.get_logger().info('catch step1: movej_p approach')

        self.create_rate(1).sleep()

        grasp_pose = self.make_pose(x, y, z, arm_state)
        self.publish_movel(grasp_pose)
        self.get_logger().info('catch step2: movel grasp')

        self.create_rate(1).sleep()

        self.close_gripper()
        self.get_logger().info('catch step3: gripper close')

        self.create_rate(1).sleep()

        lift_pose = self.make_pose(x, y, z + self.lift_height, arm_state)
        self.publish_movel(lift_pose)
        self.get_logger().info('catch step4: lift')

        self.busy = False


def main():
    rclpy.init()
    node = GraspCoordinatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 任务 6：实现 ROS2 launch 文件

创建：

```text
/workspace/RM/workspaces/rm_yolo_ros2_ws/src/vi_grab/launch/vi_grab_demo.launch.py
```

内容：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    enable_motion = LaunchConfiguration('enable_motion')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt'
        ),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false'
        ),

        Node(
            package='vi_grab',
            executable='object_detector',
            name='object_detect',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'confidence': 0.5,
                'show_image': True,
                'color_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
            }]
        ),

        Node(
            package='vi_grab',
            executable='grasp_coordinator',
            name='object_catch',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'approach_offset': 0.07,
                'lift_height': 0.05,
                'speed': 20,
                'enable_motion': enable_motion,
            }]
        ),
    ])
```

---

## 任务 7：修改 `vi_grab/setup.py`

确保 `setup.py` 包含：

```python
from setuptools import setup
import os
from glob import glob

package_name = 'vi_grab'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='YOLOv8 visual grasp demo for RM65 on ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_detector = vi_grab.object_detector_node:main',
            'grasp_coordinator = vi_grab.grasp_coordinator_node:main',
        ],
    },
)
```

---

## 任务 8：修改 `vi_grab/package.xml`

确保包含运行依赖：

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>message_filters</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
<exec_depend>vi_msgs</exec_depend>
<exec_depend>rm_ros_interfaces</exec_depend>
```

---

## 任务 9：安装缺失依赖

在容器里执行：

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash || true

apt update
apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-control-msgs

pip3 install ultralytics scipy numpy opencv-python
```

---

## 任务 10：编译并验证

执行：

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select vi_msgs
source install/setup.bash

ros2 interface show vi_msgs/msg/ObjectInfo

colcon build
source install/setup.bash
```

如果编译失败：

1. 读取完整错误。
2. 优先检查 `rm_ros_interfaces` 消息字段。
3. 用 `ros2 interface show` 检查真实接口。
4. 修复代码。
5. 重新编译。

---

## 任务 11：给出运行命令

完成代码和编译后，在终端输出以下运行说明：

### 终端 1：启动 RM65

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws
source install/setup.bash
ros2 launch rm_bringup rm_65_bringup.launch.py
```

### 终端 2：启动 D435

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 终端 3：启动视觉抓取，先不要真实运动

```bash
cd /workspace/RM/workspaces/rm_yolo_ros2_ws
source install/setup.bash
ros2 launch vi_grab vi_grab_demo.launch.py enable_motion:=false
```

### 终端 4：发布抓取目标

```bash
ros2 topic pub --once /choice_object std_msgs/msg/String "{data: 'bottle'}"
```

确认坐标、TF、机械臂状态正常后，才允许：

```bash
ros2 launch vi_grab vi_grab_demo.launch.py enable_motion:=true
```

---

## 任务 12：安全要求

必须保留 `enable_motion=false` 作为默认值。

真实机械臂运动前必须满足：

* 急停可用
* 机械臂工作空间无人
* 夹爪不会撞击桌面、相机或机械臂
* 已确认 `/object_pose` 坐标不是 0、不是 NaN、不是背景点
* 已确认 TF 链路正确

---

## 任务 13：不要做的事

不要：

* 不要把 `/workspace/RM/references/original_yolov8_ros1` 复制进 `/workspace/RM/workspaces/rm_yolo_ros2_ws/src`
* 不要用 `catkin build`
* 不要用 `roslaunch`
* 不要用 `rostopic`
* 不要重写 RM 底层驱动
* 不要默认启用真实机械臂运动
* 不要在 subscriber 回调里使用长时间阻塞逻辑
* 不要臆造 `rm_ros_interfaces` 消息字段，必须用 `ros2 interface show` 验证

---

请直接修改文件、编译、根据错误修复，直到 `colcon build` 通过，并最后输出新增/修改了哪些文件，以及如何运行。

```
::contentReference[oaicite:0]{index=0}
```
