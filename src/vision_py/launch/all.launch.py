from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vision_py'), 'launch'),
            '/zed2iPublish.launch.py'])
    )

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_robot_driver'), 'launch'),
            '/ur_control.launch.py']),
        launch_arguments={'ur_type': 'ur10e', 'robot_ip':'192.168.1.24', 'launch_rviz':'false'}.items()
    )

    calibration_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vision_py'), 'launch'),
            '/eyeInHandCalib.launch.py'])
    )

    return LaunchDescription([
        camera_node,
        robot_node,
        calibration_node
    ])
