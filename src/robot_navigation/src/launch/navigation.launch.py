import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    localization_mode = LaunchConfiguration('localization_mode', default='amcl')
    
    # 2D Map for Nav2 (Costmaps)
    map_yaml_path = LaunchConfiguration('map', default='/home/suja/voxel_ws/src/robot_navigation/src/map/1126.yaml')
    
    # 3D Map for ICP (Localization)
    pcd_file_path = LaunchConfiguration('pcd_file', default='/home/suja/voxel_ws/test.pcd')
    
    params_file = LaunchConfiguration('params_file', default=os.path.join(robot_navigation_dir, 'params', 'nav2_params.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')

    # 1. Localization Launch (ICP or AMCL)
    # We disable RViz here because we launch it at the top level
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_file': pcd_file_path,
            'use_rviz': 'false', # Disable RViz here as we launch it explicitly below
            'localization_mode': localization_mode,
            'map': map_yaml_path,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # 2. Explicit Nav2 Nodes Launch (Replacing nav2_bringup)
    
    # Lifecycle nodes to be managed
    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'smoother_server'
    ]
    
    lifecycle_nodes_localization = ['map_server']

    # Map Server
    # Only launch if localization_mode is 'icp' (because AMCL mode in bringup already launches it)
    map_server_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'icp'"])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml_path}, {'use_sim_time': use_sim_time}]
    )

    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    # Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager for Localization (Map Server)
    # Only launch if localization_mode is 'icp'
    lifecycle_manager_localization_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'icp'"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes_localization}
        ]
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes_navigation}
        ]
    )

    # 3. RViz
    # 3. RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "rviz", "nav2_default_view.rviz"]
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        remappings=[('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value='/home/suja/voxel_ws/src/robot_navigation/src/map/1126.yaml', description='Path to 2D Map YAML file for Nav2'),
        DeclareLaunchArgument('pcd_file', default_value='/home/suja/voxel_ws/test.pcd', description='Path to 3D PCD map file for ICP'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(robot_navigation_dir, 'params', 'nav2_params.yaml'), description='Nav2 parameters'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RVIZ'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('localization_mode', default_value='amcl', description='Localization mode: icp or amcl'),

        localization_launch,
        
        # Nav2 Nodes
        map_server_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        smoother_server_node,
        
        # Lifecycle Managers
        lifecycle_manager_localization_node,
        lifecycle_manager_navigation_node,
        

        
        rviz_node
    ])
