import imp
from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    zed2i_publisher = Node(
        package="vision_py",
        executable="zed2i_publisher"
    )
    calib_eyeinhand = Node(
        package="vision_py",
        executable="calib_eyeinhand"
    )

    launch_description= LaunchDescription(
        [zed2i_publisher,
         calib_eyeinhand]
    )

    return launch_description