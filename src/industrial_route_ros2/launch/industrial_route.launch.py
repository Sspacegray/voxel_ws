from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("industrial_route_ros2")
    default_params = PathJoinSubstitution([pkg_share, "config", "industrial_route.yaml"])
    default_graph = PathJoinSubstitution([pkg_share, "config", "sample_graph.geojson"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("graph_geojson_path", default_value=default_graph),
            Node(
                package="industrial_route_ros2",
                executable="industrial_route_server",
                name="industrial_route_server",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"graph_geojson_path": LaunchConfiguration("graph_geojson_path")},
                ],
            ),
        ]
    )
