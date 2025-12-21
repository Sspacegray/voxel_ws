from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("waypoint_editor")
    default_rviz_config = os.path.join(pkg, "rviz", "rviz_waypoint_editor.rviz")

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz config that includes waypoint_editor panel/tool",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_waypoint_editor",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription([declare_rviz_config, rviz2])

