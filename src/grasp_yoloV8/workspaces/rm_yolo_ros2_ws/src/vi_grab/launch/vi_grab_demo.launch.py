from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    enable_motion = LaunchConfiguration('enable_motion')
    gripper_backend = LaunchConfiguration('gripper_backend')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt'
        ),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'gripper_backend',
            default_value='rm_driver'
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
                'gripper_backend': gripper_backend,
                'pregrasp_open': True,
                'motion_wait_sec': 1.0,
                'gripper_wait_sec': 1.0,
                'rm_gripper_open_position': 1000,
                'rm_gripper_pick_speed': 200,
                'rm_gripper_pick_force': 200,
                'rm_gripper_timeout_ms': 1000,
                'motor_topic': '/motor_control',
                'motor_id': 1,
                'motor_speed': 200,
                'motor_open_dir': 0,
                'motor_close_dir': 1,
                'motor_mode': 2,
                'motor_angle': 30000,
                'motor_state': 0,
                'motor_sub_divide': 32,
            }]
        ),
    ])
