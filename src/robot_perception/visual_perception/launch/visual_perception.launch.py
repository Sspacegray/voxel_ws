#!/usr/bin/env python3
"""
Visual Perception Launch File

Launches all visual perception nodes for D435 depth camera:
- depth_processor_node: Filters depth images, generates obstacle point cloud
- depth_to_laser_node: Converts depth to virtual laser scan for costmap
- road_segmentation_node: Identifies traversable ground areas
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('visual_perception')
    
    # Default parameter file
    default_params_file = os.path.join(pkg_dir, 'config', 'visual_perception_params.yaml')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to visual perception parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='Camera namespace for topic remapping'
    )

    enable_depth_processor_arg = DeclareLaunchArgument(
        'enable_depth_processor',
        default_value='true',
        description='Enable depth processor node'
    )

    enable_depth_to_laser_arg = DeclareLaunchArgument(
        'enable_depth_to_laser',
        default_value='true',
        description='Enable depth to laser node'
    )

    enable_road_segmentation_arg = DeclareLaunchArgument(
        'enable_road_segmentation',
        default_value='true',
        description='Enable road segmentation node'
    )

    # Get launch configurations
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_namespace = LaunchConfiguration('camera_namespace')

    # Common remappings for all nodes
    depth_topic_remap = ('camera/depth/image_rect_raw', 
                          [camera_namespace, '/depth/image_rect_raw'])
    color_topic_remap = ('camera/color/image_raw', 
                          [camera_namespace, '/color/image_raw'])
    camera_info_remap = ('camera/depth/camera_info', 
                          [camera_namespace, '/depth/camera_info'])

    # Depth Processor Node
    depth_processor_node = Node(
        package='visual_perception',
        executable='depth_processor_node',
        name='depth_processor',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('camera/depth/image_rect_raw', [camera_namespace, '/depth/image_rect_raw']),
            ('camera/depth/camera_info', [camera_namespace, '/depth/camera_info']),
        ],
    )

    # Depth to Laser Node
    depth_to_laser_node = Node(
        package='visual_perception',
        executable='depth_to_laser_node',
        name='depth_to_laser',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('camera/depth/image_rect_raw', [camera_namespace, '/depth/image_rect_raw']),
            ('camera/depth/camera_info', [camera_namespace, '/depth/camera_info']),
        ],
    )

    # Road Segmentation Node
    road_segmentation_node = Node(
        package='visual_perception',
        executable='road_segmentation_node',
        name='road_segmentation',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('camera/depth/image_rect_raw', [camera_namespace, '/depth/image_rect_raw']),
            ('camera/color/image_raw', [camera_namespace, '/color/image_raw']),
            ('camera/depth/camera_info', [camera_namespace, '/depth/camera_info']),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        camera_namespace_arg,
        enable_depth_processor_arg,
        enable_depth_to_laser_arg,
        enable_road_segmentation_arg,
        depth_processor_node,
        depth_to_laser_node,
        road_segmentation_node,
    ])
