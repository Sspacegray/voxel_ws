#!/usr/bin/env python3
#
# Integrated simulation + Nav2 + Waypoint Editor (FollowPath) + Route Server.
#
# - Simulator: wpr_simulation2/launch/robocup_home.launch.py
# - Nav2: nav2_bringup/launch/bringup_launch.py
# - RViz: waypoint_editor/rviz/rviz_waypoint_editor.rviz
# - Route network: robot_route/launch/route_demo.launch.py (optional)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wpr_dir = get_package_share_directory('wpr_simulation2')
    nav2_dir = get_package_share_directory('nav2_bringup')
    waypoint_editor_dir = get_package_share_directory('waypoint_editor')
    robot_route_dir = get_package_share_directory('robot_route')

    default_map = os.path.join(wpr_dir, 'maps', 'my_map.yaml')
    default_nav2_params = os.path.join(wpr_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(waypoint_editor_dir, 'rviz', 'rviz_waypoint_editor.rviz')
    default_route_graph = os.path.join(robot_route_dir, 'maps', '1126.geojson')
    default_route_params = os.path.join(robot_route_dir, 'config', 'route_server.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to Nav2 params YAML',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to RViz config (contains waypoint_editor panel/tool)',
    )
    declare_enable_route = DeclareLaunchArgument(
        'enable_route',
        default_value='False',
        description='Start route_server (nav2_route) for route network visualization/planning',
    )
    declare_route_graph = DeclareLaunchArgument(
        'route_graph',
        default_value=default_route_graph,
        description='Full path to the route graph geojson file',
    )
    declare_route_params = DeclareLaunchArgument(
        'route_params_file',
        default_value=default_route_params,
        description='Full path to route_server params YAML',
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM',
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_route = LaunchConfiguration('enable_route')
    route_graph = LaunchConfiguration('route_graph')
    route_params_file = LaunchConfiguration('route_params_file')
    slam = LaunchConfiguration('slam')

    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(wpr_dir, 'launch', 'robocup_home.launch.py'))
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam': slam,
        }.items(),
    )

    route_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_route_dir, 'launch', 'route_demo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': route_params_file,
            'graph': route_graph,
        }.items(),
        condition=IfCondition(enable_route),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_enable_route)
    ld.add_action(declare_route_graph)
    ld.add_action(declare_route_params)

    ld.add_action(simulation_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(route_cmd)
    ld.add_action(rviz_cmd)

    return ld
