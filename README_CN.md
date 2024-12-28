# Chassis Cybergear ros2

这是个使用小米cybergear 电机的底盘控制程序，集成速度控制，里程计反馈，电量反馈等功能。使用socketcan 协议通信

## 环境
ros2 foxy 已经测试

## 安装依赖
``` bash
git clone git@github.com:Josme/chassis_cybergear_ros2.git && cd chassis_cybergear_ros2
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## 编译
``` bash
colcon build 
```

## 运行

``` bash
source install/setup.bash
ros2 launch chassis_cybergear_ros2 chassis_control.launch.py
```
