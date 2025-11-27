import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    icp_dir = get_package_share_directory('icp_registration')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Renamed argument to avoid conflict with Nav2 'map' argument
    pcd_path = LaunchConfiguration('pcd_file', default='/home/suja/voxel_ws/test.pcd') 
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    localization_mode = LaunchConfiguration('localization_mode', default='icp')
    map_yaml_file = LaunchConfiguration('map', default='')
    params_file = LaunchConfiguration('params_file', default='')
    autostart = LaunchConfiguration('autostart', default='true')

    # 1. Bringup (Base + Fast-LIO + Linefit + Point2Scan)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization_mode': localization_mode,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # 2. ICP Registration (Localization)
    # Only launch if localization_mode is 'icp'
    icp_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'icp'"])),
        package='icp_registration',
        executable='icp_registration_node',
        output='screen',
        parameters=[
            os.path.join(icp_dir, 'config', 'icp.yaml'),
            {'pcd_path': pcd_path} # Use the renamed argument
        ]
    )

    # 3. RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'rviz_config': os.path.join(robot_navigation_dir, 'rviz', 'nav2_default_view.rviz')
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pcd_file', default_value='/home/suja/voxel_ws/test.pcd', description='Path to PCD map file for ICP'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RVIZ'),
        DeclareLaunchArgument('localization_mode', default_value='amcl', description='Localization mode: icp or amcl'),
        DeclareLaunchArgument('map', default_value='', description='Path to map yaml for AMCL'),
        DeclareLaunchArgument('params_file', default_value='', description='Nav2 parameters file'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2 nodes'),
        
        bringup_launch,
        icp_node,
        rviz_launch
    ])
