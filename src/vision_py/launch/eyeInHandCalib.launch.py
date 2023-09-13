from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vision_py",
            executable="calib_eyeinhand",
            name="eyeInHand_Calibrator",
            parameters=[{
                'image_topic': '/my_zed2i/left_image_raw',
                'info_topic': '/my_zed2i/left_camera_info',
                'base_link': 'base',
                'gripper_link': 'tool0'
            }]
        ),
    ])
