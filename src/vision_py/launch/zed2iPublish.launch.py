from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vision_py",
            executable="zed2i_publisher",
            name="zed2i_Publisher"
        ),
    ])
