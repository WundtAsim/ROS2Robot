# robot
start the zed2i
```
ros2 launch zed_wrapper zed2i.launch.py
```

start the driver
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.1.24 launch_rviz:=false
```
Then load the URCap and play the program.

launch the moveit
```
source ~/Documents/Project/UR10e_WS/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```
