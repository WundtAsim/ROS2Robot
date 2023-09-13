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

    calibration_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('vision_py'), 'launch'),
        '/eyeInHandCalib.launch.py'])
    )

    return LaunchDescription([
        camera_node,
        calibration_node
    ])
