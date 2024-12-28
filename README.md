# Chassis Cybergear ros2

This is a chassis control program that uses Xiaomi cybergear motors, integrating speed control, odometer feedback, power feedback and other functions. Use socketcan protocol communication

## Environment
ros2 foxy has been tested

## Installation dependencies
``` bash
git clone git@github.com:Josme/chassis_cybergear_ros2.git && cd chassis_cybergear_ros2
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## Compile
``` bash
colcon build
```

## Run

``` bash
source install/setup.bash
ros2 launch chassis_cybergear_ros2 chassis_control.launch.py
```
