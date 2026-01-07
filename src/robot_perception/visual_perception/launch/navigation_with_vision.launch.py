#!/usr/bin/env python3
"""
Navigation with Vision Launch File

Complete navigation system that integrates:
- D435 RealSense camera for visual perception
- Visual obstacle detection (depth to laser scan)
- Road segmentation for traversability analysis
- Nav2 navigation stack with visual costmap layer
- Lidar + vision sensor fusion for robust obstacle avoidance
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    visual_perception_dir = get_package_share_directory('visual_perception')
    
    # Try to get wpr_simulation2 for simulation
    try:
        wpr_simulation_dir = get_package_share_directory('wpr_simulation2')
        has_simulation = True
    except Exception:
        has_simulation = False
        wpr_simulation_dir = ''
    
    # Try to get nav2_bringup
    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        has_nav2 = True
    except Exception:
        has_nav2 = False
        nav2_bringup_dir = ''

    # ===========================================================================
    # Declare Launch Arguments
    # ===========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='Camera namespace'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable D435 camera launch'
    )

    enable_visual_perception_arg = DeclareLaunchArgument(
        'enable_visual_perception',
        default_value='true',
        description='Enable visual perception nodes'
    )

    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Enable Nav2 navigation stack (set to true if not using existing nav launch)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='',
        description='Path to Nav2 params file (with visual obstacle layer)'
    )

    visual_params_arg = DeclareLaunchArgument(
        'visual_params_file',
        default_value=os.path.join(visual_perception_dir, 'config', 'visual_perception_params.yaml'),
        description='Path to visual perception params file'
    )

    # ===========================================================================
    # Get Launch Configurations
    # ===========================================================================
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_namespace = LaunchConfiguration('camera_namespace')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_visual_perception = LaunchConfiguration('enable_visual_perception')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    visual_params_file = LaunchConfiguration('visual_params_file')

    # ===========================================================================
    # D435 Camera Launch
    # ===========================================================================
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(visual_perception_dir, 'launch', 'd435_camera.launch.py')
        ]),
        launch_arguments={
            'camera_name': camera_namespace,
        }.items(),
        condition=IfCondition(enable_camera),
    )

    # ===========================================================================
    # Visual Perception Nodes Launch
    # ===========================================================================
    
    visual_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(visual_perception_dir, 'launch', 'visual_perception.launch.py')
        ]),
        launch_arguments={
            'params_file': visual_params_file,
            'use_sim_time': use_sim_time,
            'camera_namespace': camera_namespace,
        }.items(),
        condition=IfCondition(enable_visual_perception),
    )

    # ===========================================================================
    # Static TF: Camera to Robot Base (adjust these for your robot!)
    # ===========================================================================
    # This publishes the transform from base_link to camera_link
    # Adjust x, y, z, roll, pitch, yaw based on your camera mounting position
    
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_tf',
        arguments=[
            '--x', '0.1',      # Camera 10cm forward from base_link
            '--y', '0.0',      # Centered
            '--z', '0.3',      # Camera 30cm above base_link
            '--roll', '0.0',
            '--pitch', '0.0',  # Tilted down slightly (in radians, e.g., 0.1 ~ 6 deg)
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        # Only publish if we're launching the camera
        condition=IfCondition(enable_camera),
    )

    # ===========================================================================
    # Return Launch Description
    # ===========================================================================
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        camera_namespace_arg,
        enable_camera_arg,
        enable_visual_perception_arg,
        enable_nav2_arg,
        map_file_arg,
        nav2_params_arg,
        visual_params_arg,
        
        # Camera
        camera_launch,
        camera_tf_node,
        
        # Visual Perception
        visual_perception_launch,
    ])
