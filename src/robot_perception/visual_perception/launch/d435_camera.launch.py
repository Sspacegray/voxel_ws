#!/usr/bin/env python3
"""
D435 RealSense Camera Launch File

Launches the Intel RealSense D435 camera driver with appropriate settings
for depth-based obstacle avoidance and road segmentation.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name (used as namespace)'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth stream'
    )

    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream'
    )

    enable_infra_arg = DeclareLaunchArgument(
        'enable_infra',
        default_value='false',
        description='Enable infrared streams'
    )

    depth_width_arg = DeclareLaunchArgument(
        'depth_width',
        default_value='640',
        description='Depth image width'
    )

    depth_height_arg = DeclareLaunchArgument(
        'depth_height',
        default_value='480',
        description='Depth image height'
    )

    depth_fps_arg = DeclareLaunchArgument(
        'depth_fps',
        default_value='30',
        description='Depth image frame rate'
    )

    color_width_arg = DeclareLaunchArgument(
        'color_width',
        default_value='640',
        description='Color image width'
    )

    color_height_arg = DeclareLaunchArgument(
        'color_height',
        default_value='480',
        description='Color image height'
    )

    color_fps_arg = DeclareLaunchArgument(
        'color_fps',
        default_value='30',
        description='Color image frame rate'
    )

    align_depth_arg = DeclareLaunchArgument(
        'align_depth',
        default_value='true',
        description='Align depth to color frame'
    )

    pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='false',
        description='Enable point cloud generation in driver (we do this in visual_perception)'
    )

    # Get realsense2_camera package path
    try:
        realsense_launch_dir = os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch'
        )
        use_realsense_launch = True
    except Exception:
        use_realsense_launch = False

    # RealSense camera node (using parameters directly)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'align_depth.enable': True,
            'pointcloud.enable': False,
            'enable_sync': True,
            'enable_rgbd': False,
            'publish_tf': True,
            'tf_publish_rate': 0.0,  # Static TF only
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        camera_name_arg,
        enable_depth_arg,
        enable_color_arg,
        enable_infra_arg,
        depth_width_arg,
        depth_height_arg,
        depth_fps_arg,
        color_width_arg,
        color_height_arg,
        color_fps_arg,
        align_depth_arg,
        pointcloud_arg,
        realsense_node,
    ])
