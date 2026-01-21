#!/usr/bin/env python3
#
# Copyright 2023 6-robot.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Zhang Wanjie

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_autostart = LaunchConfiguration('nav2_autostart', default='true')
    
    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')

    home_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robocup_home.launch.py')
        )
    )

    behavior_tree = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'behavior_tree.xml'
    )

    map_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'maps',
        'my_map.yaml'
    )
    
    nav_param_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'nav2_params.yaml'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav_param_file,
        description='Full path to Nav2 params YAML (controller_server config lives here)',
    )
    params_file = LaunchConfiguration('params_file')
    declare_nav2_autostart = DeclareLaunchArgument(
        'nav2_autostart',
        default_value='true',
        description='Auto-start Nav2 lifecycle nodes (navigation stack)',
    )

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'launch'
    )

    navigation_cmd = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_engine'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': nav2_autostart,
                }.items(),
            )
        ]
    )

    waypoint_editor_dir = get_package_share_directory('waypoint_editor')
    rviz_config = os.path.join(waypoint_editor_dir, 'rviz', 'rviz_waypoint_editor.rviz')

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            # Force X11 backend to avoid Wayland/Xwayland GPU contention issues with Gazebo
            additional_env={'QT_QPA_PLATFORM': 'xcb'}
        )

    # Trajectory recorder node for path tracking analysis
    trajectory_recorder_cmd = Node(
        package='robot_route',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ============================================================
    # Industrial Route Server - 工业路网导航
    # ============================================================
    # 默认路网文件路径 (修改此处加载不同的路网)
    default_route_graph = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'maps',
        'robocup_home_route.geojson'
    )
    
    # Industrial Route 参数文件
    industrial_route_params = os.path.join(
        get_package_share_directory('industrial_route_ros2'),
        'config',
        'industrial_route.yaml'
    )
    
    declare_route_graph = DeclareLaunchArgument(
        'route_graph',
        default_value=default_route_graph,
        description='Path to route network GeoJSON file'
    )
    route_graph = LaunchConfiguration('route_graph')
    
    declare_enable_route = DeclareLaunchArgument(
        'enable_route',
        default_value='true',
        description='Enable industrial route server'
    )
    enable_route = LaunchConfiguration('enable_route')

    # Industrial Route Server 节点
    industrial_route_cmd = Node(
        package='industrial_route_ros2',
        executable='industrial_route_server',
        name='industrial_route_server',
        output='screen',
        condition=IfCondition(enable_route),
        parameters=[
            industrial_route_params,
            {
                'graph_geojson_path': route_graph,
                'use_sim_time': True,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'cmd_vel_topic': '/cmd_vel_engine',  # 与 Nav2 一致
                'use_map_collision_check': False,
            }
        ],
    )

    # PP Controller waypoint file parameter
    # ====== 修改此处来加载不同的路径文件 ======
    waypoint_file_default = ''
    
    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value=waypoint_file_default,
        description='Path to waypoint JSON file for PP Controller'
    )
    waypoint_file = LaunchConfiguration('waypoint_file')

    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start path execution when waypoint_file is specified'
    )
    auto_start = LaunchConfiguration('auto_start')

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_params_file)
    ld.add_action(declare_nav2_autostart)
    ld.add_action(declare_waypoint_file)
    ld.add_action(declare_auto_start)
    ld.add_action(declare_route_graph)
    ld.add_action(declare_enable_route)
    ld.add_action(home_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(trajectory_recorder_cmd)
    ld.add_action(industrial_route_cmd)  # 工业路网服务器

    return ld

