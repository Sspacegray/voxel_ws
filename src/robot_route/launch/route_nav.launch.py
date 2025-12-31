"""
路网导航集成启动文件 (Independent Version)

一键启动：仿真 + Nav2 + 路网服务 + 桥接节点
不依赖 wpr_simulation2/navigation.launch.py，直接调用 nav2_bringup 以确保参数传递正确。

使用方法:
    ros2 launch robot_route route_nav.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package directories
    robot_route_dir = get_package_share_directory('robot_route')
    wpr_sim_dir = get_package_share_directory('wpr_simulation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    waypoint_editor_dir = get_package_share_directory('waypoint_editor')
    
    # 1. Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Map: use my_map.yaml from wpr_simulation2
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(wpr_sim_dir, 'maps', 'my_map.yaml')
    )
    
    # Route Graph
    graph_filepath = LaunchConfiguration(
        'graph',
        default=os.path.join(robot_route_dir, 'maps', 'route_graph.geojson')
    )
    
    # Params File: use wpr_simulation2/nav2_params.yaml
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(wpr_sim_dir, 'config', 'nav2_params.yaml')
    )
    
    # Rviz Config
    rviz_config_file = os.path.join(waypoint_editor_dir, 'rviz', 'rviz_waypoint_editor.rviz')

    # 2. Declare Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(wpr_sim_dir, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file'
    )
    
    declare_graph = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(robot_route_dir, 'maps', 'route_graph.geojson'),
        description='Path to GeoJSON route graph file'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(wpr_sim_dir, 'config', 'nav2_params.yaml'),
        description='Path to param file'
    )
    
    # 3. Simulation Launch (Robot + Gazebo)
    # Does NOT launch Nav2 or Rviz
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wpr_sim_dir, 'launch', 'robocup_home.launch.py')
        )
    )
    
    # 4. Nav2 Bringup (Controller, Planner, Beacon, etc.)
    # Explicitly wrapping in GroupAction to remap cmd_vel
    nav2_cmd = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_engine'),
            SetRemap(src='/goal_pose', dst='/goal_pose_disabled'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file
                }.items()
            )
        ]
    )
    
    # 5. Route Server
    route_server_params = RewrittenYaml(
        source_file=os.path.join(robot_route_dir, 'config', 'route_server.yaml'),
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    route_server_node = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        parameters=[route_server_params, {'graph_filepath': graph_filepath}]
    )
    
    route_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_route',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'bond_timeout': 100.0},
            {'node_names': ['route_server']}
        ]
    )
    
    # 6. Route Goal Bridge
    route_goal_bridge_node = Node(
        package='robot_route',
        executable='route_goal_bridge.py',
        name='route_goal_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'first_mile_threshold': 0.5},
            {'last_mile_threshold': 0.5},
            {'fallback_to_nav': False}
        ]
    )
    
    # 7. Rviz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        output='screen'
    )
    
    # 8. Trajectory Recorder (Optional but useful)
    trajectory_recorder_cmd = Node(
        package='robot_route',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Environment
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    
    # Arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_graph)
    ld.add_action(declare_params_file)
    
    # Executables
    ld.add_action(sim_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(route_server_node)
    ld.add_action(route_lifecycle_manager)
    ld.add_action(route_goal_bridge_node)
    ld.add_action(rviz_cmd)
    ld.add_action(trajectory_recorder_cmd)
    
    return ld
