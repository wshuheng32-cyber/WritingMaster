import math
import threading
import time
from importlib import import_module

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import Bool, Empty, String

from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from rm_ros_interfaces.msg import Armstate, Gripperpick, Gripperset, Movejp, Movel
from vi_msgs.msg import ObjectInfo


class GraspCoordinatorNode(Node):
    def __init__(self):
        super().__init__('object_catch')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('approach_offset', 0.15)
        self.declare_parameter('lift_height', 0.05)
        self.declare_parameter('speed', 3)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('gripper_backend', 'rm_driver')
        self.declare_parameter('pregrasp_open', True)
        self.declare_parameter('motion_wait_sec', 1.0)
        self.declare_parameter('gripper_wait_sec', 1.0)
        self.declare_parameter('rm_gripper_open_position', 1000)
        self.declare_parameter('rm_gripper_pick_speed', 200)
        self.declare_parameter('rm_gripper_pick_force', 200)
        self.declare_parameter('rm_gripper_timeout_ms', 1000)
        self.declare_parameter('motor_topic', '/motor_control')
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('motor_speed', 200)
        self.declare_parameter('motor_open_dir', 0)
        self.declare_parameter('motor_close_dir', 1)
        self.declare_parameter('motor_mode', 2)
        self.declare_parameter('motor_angle', 30000)
        self.declare_parameter('motor_state', 0)
        self.declare_parameter('motor_sub_divide', 32)

        self.base_frame = self.get_parameter('base_frame').value
        self.approach_offset = float(self.get_parameter('approach_offset').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.speed = int(self.get_parameter('speed').value)
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.gripper_backend = str(self.get_parameter('gripper_backend').value)
        self.pregrasp_open = bool(self.get_parameter('pregrasp_open').value)
        self.motion_wait_sec = float(self.get_parameter('motion_wait_sec').value)
        self.gripper_wait_sec = float(self.get_parameter('gripper_wait_sec').value)
        self.rm_gripper_open_position = int(self.get_parameter('rm_gripper_open_position').value)
        self.rm_gripper_pick_speed = int(self.get_parameter('rm_gripper_pick_speed').value)
        self.rm_gripper_pick_force = int(self.get_parameter('rm_gripper_pick_force').value)
        self.rm_gripper_timeout_ms = int(self.get_parameter('rm_gripper_timeout_ms').value)
        self.motor_topic = str(self.get_parameter('motor_topic').value)
        self.motor_id = int(self.get_parameter('motor_id').value)
        self.motor_speed = int(self.get_parameter('motor_speed').value)
        self.motor_open_dir = int(self.get_parameter('motor_open_dir').value)
        self.motor_close_dir = int(self.get_parameter('motor_close_dir').value)
        self.motor_mode = int(self.get_parameter('motor_mode').value)
        self.motor_angle = int(self.get_parameter('motor_angle').value)
        self.motor_state = int(self.get_parameter('motor_state').value)
        self.motor_sub_divide = int(self.get_parameter('motor_sub_divide').value)

        self.target_class = ''
        self.latest_arm_state = None
        self.busy = False
        self.lock = threading.Lock()
        self.motor_msg_type = None

        self._movejp_result_event = threading.Event()
        self._movel_result_event = threading.Event()
        self._movejp_result_ok = False
        self._movel_result_ok = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(String, '/choice_object', self.choice_callback, 10)
        self.create_subscription(ObjectInfo, '/object_pose', self.object_callback, 10)
        self.create_subscription(
            Armstate,
            '/rm_driver/get_current_arm_state_result',
            self.arm_state_callback,
            10
        )
        self.create_subscription(Bool, '/rm_driver/movej_p_result', self._movejp_result_cb, 10)
        self.create_subscription(Bool, '/rm_driver/movel_result', self._movel_result_cb, 10)

        self.arm_state_req_pub = self.create_publisher(
            Empty,
            '/rm_driver/get_current_arm_state_cmd',
            10
        )
        self.movejp_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.movel_pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.gripper_pub = self.create_publisher(
            Gripperpick,
            '/rm_driver/set_gripper_pick_cmd',
            10
        )
        self.gripper_position_pub = self.create_publisher(
            Gripperset,
            '/rm_driver/set_gripper_position_cmd',
            10
        )

        self.motor_pub = None
        if self.gripper_backend == 'step_motor':
            try:
                self.motor_msg_type = import_module('step_motor.msg').Motor
            except ImportError as exc:
                raise RuntimeError(
                    'gripper_backend=step_motor but step_motor.msg is unavailable. '
                    'Source /workspace/RM/workspaces/step_motor_ros2_ws/install/setup.bash first.'
                ) from exc

            self.motor_pub = self.create_publisher(
                self.motor_msg_type,
                self.motor_topic,
                10
            )
        elif self.gripper_backend != 'rm_driver':
            raise ValueError(
                f'Unsupported gripper_backend: {self.gripper_backend}. '
                'Expected one of: rm_driver, step_motor'
            )

        self.create_timer(0.2, self.request_arm_state)

        self.get_logger().warn(
            f'Grasp coordinator started. enable_motion={self.enable_motion}, '
            f'gripper_backend={self.gripper_backend}. '
            '调试确认无误后再打开真实运动。'
        )

    def request_arm_state(self):
        self.arm_state_req_pub.publish(Empty())

    def arm_state_callback(self, msg: Armstate):
        self.latest_arm_state = msg

    def _movejp_result_cb(self, msg: Bool):
        self._movejp_result_ok = msg.data
        self._movejp_result_event.set()

    def _movel_result_cb(self, msg: Bool):
        self._movel_result_ok = msg.data
        self._movel_result_event.set()

    def _wait_movejp(self, timeout=15.0) -> bool:
        self._movejp_result_event.clear()
        self._movejp_result_ok = False
        if not self._movejp_result_event.wait(timeout):
            self.get_logger().error('movej_p 等待超时')
            return False
        return self._movejp_result_ok

    def _wait_movel(self, timeout=15.0) -> bool:
        self._movel_result_event.clear()
        self._movel_result_ok = False
        if not self._movel_result_event.wait(timeout):
            self.get_logger().error('movel 等待超时')
            return False
        return self._movel_result_ok

    def choice_callback(self, msg: String):
        with self.lock:
            self.target_class = msg.data.strip()
            self.busy = False
        self.get_logger().info(f'收到抓取目标: {self.target_class}')

    def object_callback(self, msg: ObjectInfo):
        with self.lock:
            if self.busy or not self.target_class:
                return

            if msg.object_class != self.target_class:
                return

            if self.latest_arm_state is None:
                self.get_logger().warn(
                    '还没有收到机械臂状态，等待 /rm_driver/get_current_arm_state_result'
                )
                return

            arm_state = self.latest_arm_state
            self.busy = True
            self.target_class = ''

        try:
            base_point = self.transform_object_to_base(msg)
        except Exception as exc:
            self.get_logger().warn(f'TF 转换失败: {exc}')
            with self.lock:
                self.busy = False
            return

        thread = threading.Thread(
            target=self.execute_grasp,
            args=(base_point, arm_state),
            daemon=True
        )
        thread.start()

    def transform_object_to_base(self, obj: ObjectInfo):
        point = PointStamped()
        point.header = obj.header
        if not point.header.frame_id:
            point.header.frame_id = 'camera_color_optical_frame'
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
        pose.orientation = arm_state.pose.orientation
        return pose

    def publish_movejp(self, pose: Pose):
        msg = Movejp()
        msg.pose = pose
        msg.speed = self.speed
        msg.trajectory_connect = 0
        msg.block = True
        self.movejp_pub.publish(msg)

    def publish_movel(self, pose: Pose):
        msg = Movel()
        msg.pose = pose
        msg.speed = self.speed
        msg.trajectory_connect = 0
        msg.block = True
        self.movel_pub.publish(msg)

    def close_gripper(self):
        if self.gripper_backend == 'step_motor':
            self.publish_motor_command(self.motor_close_dir)
            return

        msg = Gripperpick()
        msg.speed = self.rm_gripper_pick_speed
        msg.force = self.rm_gripper_pick_force
        msg.block = True
        msg.timeout = self.rm_gripper_timeout_ms
        self.gripper_pub.publish(msg)

    def open_gripper(self):
        if self.gripper_backend == 'step_motor':
            self.publish_motor_command(self.motor_open_dir)
            return

        msg = Gripperset()
        msg.position = self.rm_gripper_open_position
        msg.block = True
        msg.timeout = self.rm_gripper_timeout_ms
        self.gripper_position_pub.publish(msg)

    def publish_motor_command(self, direction: int):
        if self.motor_pub is None or self.motor_msg_type is None:
            raise RuntimeError('step_motor publisher is not initialized')

        msg = self.motor_msg_type()
        msg.id = self.motor_id
        msg.speed = self.motor_speed
        msg.dir = direction
        msg.mode = self.motor_mode
        msg.angle = self.motor_angle
        msg.state = self.motor_state
        msg.sub_divide = self.motor_sub_divide
        self.motor_pub.publish(msg)

    def execute_grasp(self, base_point: PointStamped, arm_state: Armstate):
        try:
            x = base_point.point.x
            y = base_point.point.y
            z = base_point.point.z

            reach = math.sqrt(x * x + y * y + z * z)
            ax = arm_state.pose.position.x
            ay = arm_state.pose.position.y
            az = arm_state.pose.position.z
            self.get_logger().info(
                f'目标 {self.base_frame} 坐标: x={x:.3f}, y={y:.3f}, z={z:.3f}  '
                f'距基座={reach:.3f}m  '
                f'末端当前: x={ax:.3f}, y={ay:.3f}, z={az:.3f}'
            )
            if reach > 0.82:
                self.get_logger().warn(
                    f'目标距基座 {reach:.3f}m，可能超出 RM65 工作空间（~0.82m 有效范围）。'
                    '请将机械臂移近目标物体后重试。'
                )

            if not self.enable_motion:
                self.get_logger().warn('enable_motion=false，仅打印目标坐标，不执行真实抓取。')
                return

            if self.pregrasp_open:
                self.open_gripper()
                self.get_logger().info('catch step0: gripper open')
                time.sleep(self.gripper_wait_sec)

            approach_pose = self.make_pose(x, y, z + self.approach_offset, arm_state)
            self.publish_movejp(approach_pose)
            self.get_logger().info(
                f'catch step1: movej_p approach → '
                f'x={x:.3f}, y={y:.3f}, z={z + self.approach_offset:.3f}'
            )
            if not self._wait_movejp():
                self.get_logger().error('step1 movej_p 失败，中止抓取 (error code 1=参数错误/臂错误, -6=臂已停止)')
                return

            grasp_pose = self.make_pose(x, y, z, arm_state)
            self.publish_movel(grasp_pose)
            self.get_logger().info(
                f'catch step2: movel grasp → x={x:.3f}, y={y:.3f}, z={z:.3f}'
            )
            if not self._wait_movel():
                self.get_logger().error('step2 movel grasp 失败，中止抓取')
                return

            self.close_gripper()
            self.get_logger().info('catch step3: gripper close')
            time.sleep(self.gripper_wait_sec)

            lift_pose = self.make_pose(x, y, z + self.lift_height, arm_state)
            self.publish_movel(lift_pose)
            self.get_logger().info(
                f'catch step4: lift → z={z + self.lift_height:.3f}'
            )
            if not self._wait_movel():
                self.get_logger().error('step4 movel lift 失败')
                return

            self.get_logger().info('抓取完成')
        finally:
            with self.lock:
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
