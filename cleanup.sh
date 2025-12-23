#!/bin/bash
echo "正在强制清理 ROS2 和 Gazebo 进程..."

# 杀掉 Gazebo 仿真环境
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f gazebo

# 杀掉 ROS2 节点和守护进程
pkill -9 -f ros2
pkill -9 -f nav2
pkill -9 -f rviz2
pkill -9 -f robot_state_publisher
pkill -9 -f component_container

# 杀掉 Python 脚本节点
pkill -9 -f trajectory_recorder.py
pkill -9 -f pp_node

# 杀掉 C++ 控制器节点
pkill -9 -f pp_controller

echo "清理完成！现在应该可以正常重新启动了。"
