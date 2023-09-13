# release
eye in hand caibration:
```
ros2 launch vision_py all.launch.py
```


# deprecated
1. start the zed2i
```
ros2 launch zed_wrapper zed2i.launch.py
```

2. start the driver
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.1.24 launch_rviz:=false
```
3. Then load the URCap and play the program.

4. launch the moveit
```
source ~/Documents/Project/UR10e_WS/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

# setup ros2&robot
1. install ros2 and robot driver:
```
sudo apt-get install ros-${ROS_DISTRO}-ur
```
2. network setup and prepare the robot
3. export calibration file(apt upgrade first)
```
ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=192.168.1.24 target_filename:="${HOME}/my_robot_calibration.yaml"
```