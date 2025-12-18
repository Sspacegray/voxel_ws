import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    robot_route_dir = get_package_share_directory('robot_route')

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    graph_filepath = LaunchConfiguration(
        'graph',
        default=os.path.join(robot_route_dir, 'maps', '1126.geojson')
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_route_dir, 'config', 'route_server.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_graph_cmd = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(robot_route_dir, 'maps', '1126.geojson'),
        description='Full path to the route graph geojson file')

    # Configure the route_server node
    param_substitutions = {
        'use_sim_time': use_sim_time}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    route_server_node = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        parameters=[configured_params, {'graph_filepath': graph_filepath}])

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_route',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'bond_timeout': 100.0},
                    {'node_names': ['route_server']}])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_graph_cmd)

    ld.add_action(route_server_node)
    ld.add_action(lifecycle_manager_node)

    return ld
