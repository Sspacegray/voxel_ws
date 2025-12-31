# WPR系列机器人ROS2仿真工具

## 介绍课程
Bilibili: [机器人操作系统 ROS2 入门教材](https://www.bilibili.com/video/BV1oz421v7tB)  
Youtube: [机器人操作系统 ROS2 入门教材](https://www.youtube.com/watch?v=j0foOvBqQTc)

## 配套教材书籍
《机器人操作系统（ROS2）入门与实践》  
![视频课程](./media/book_1.jpg)
淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)

## 系统版本

- ROS2 Humble (Ubuntu 22.04)

## 使用说明

### 一、 启智ROS机器人
1. 获取源码:
```
cd ~/ros2_ws/src/
git clone https://github.com/6-robot/wpr_simulation2.git
```
2. 安装依赖项:  
ROS2 Humble (Ubuntu 22.04)
```
cd ~/ros2_ws/src/wpr_simulation2/scripts
./install_for_humble.sh
```
3. 编译
```
cd ~/ros2_ws
colcon build --symlink-install
```

简单场景:
```
ros2 launch wpr_simulation2 wpb_simple.launch.py 
```
![wpb_simple pic](./media/wpb_simple.png)

SLAM环境地图创建:
```
ros2 launch wpr_simulation2 slam.launch.py 
ros2 run rqt_robot_steering rqt_robot_steering 
```
![wpb_gmapping pic](./media/wpb_gmapping.png)

Navigation导航:
```
ros2 launch wpr_simulation2 navigation.launch.py
```
![wpb_navigation pic](./media/wpb_navigation.png)

集成启动（仿真 + Nav2 + Waypoint Editor 固定路径 FollowPath + 路网 route_server）:
```
ros2 launch wpr_simulation2 integrated_waypoint_nav.launch.py
```

集成启动（推荐：仿真 + Nav2 + RViz + half_structure 路网约束导航，不启动 route_server）:
```
ros2 launch wpr_simulation2 integrated_half_structure_nav.launch.py
```

控制器说明与替换:

- 仿真默认使用的 Nav2 控制器在 `wpr_simulation2/config/nav2_params.yaml` 的
  `controller_server.ros__parameters.FollowPath.plugin` 配置项里（默认是 DWB）。
- 你可以通过覆盖 `params_file` 来替换控制器（例如切到你的 PP 或 Nav2 RPP）:

```
ros2 launch wpr_simulation2 navigation.launch.py params_file:=/abs/path/to/nav2_params.yaml
```
