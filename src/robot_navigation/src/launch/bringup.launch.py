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
    robot_base_dir = get_package_share_directory('robot_base')
    fast_lio_dir = get_package_share_directory('fast_lio')
    linefit_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mapping_mode = LaunchConfiguration('mapping', default='false') # Toggle PCD saving

    # 1. Robot Base Bringup (Hardware, Controllers, TF)
    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_base_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 1.1 Livox Driver (Explicitly added as it might be commented out in base bringup)
    # We launch it here to ensure Fast-LIO gets data.
    # Note: If base bringup already launches it, this might conflict. 
    # But based on analysis, it was commented out.
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 4},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': PathJoinSubstitution([
                livox_driver_dir,
                "config",
                "MID360_config.json"
            ])},
            {'cmdline_input_bd_code': 'livox0000000001'}
        ],
        remappings=[
            ('/livox/lidar', '/livox/lidar') # Default
        ]
    )

    # 2. Fast-LIO (Odometry & Mapping/Pointcloud)
    # Launching node directly to allow parameter override for pcd_save
    fast_lio_config_path = os.path.join(fast_lio_dir, 'config', 'mid360.yaml')
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            fast_lio_config_path,
            {'use_sim_time': use_sim_time},
            {'pcd_save.pcd_save_en': mapping_mode} # Override pcd_save_en based on argument
        ],
        output='screen'
    )

    # 3. Linefit Ground Segmentation
    linefit_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[
            os.path.join(linefit_dir, 'launch', 'segmentation_params.yaml'),
            {'input_topic': '/cloud_registered_body'} # Override input topic
        ],
        remappings=[
            ('ground_segmentation/obstacle_cloud', '/segmentation/obstacle')
        ]
    )

    # 4. Pointcloud to LaserScan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        remappings=[
            ('cloud_in', '/segmentation/obstacle'),
            ('scan', '/scan')
        ]
    )

    # Static TF: odom -> camera_init (Fast-LIO World Frame)
    # Assumes Fast-LIO starts at odom origin
    static_tf_odom_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF: body -> base_link (Fast-LIO Body Frame)
    # Assumes Fast-LIO body frame aligns with robot base_link
    # If Fast-LIO body is IMU, and IMU is at base_link, this is correct.
    # If IMU is offset, this should be the inverse of base_link->imu_link, 
    # BUT usually we treat Fast-LIO body as the base for navigation.
    static_tf_body_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_base',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. AMCL Localization (Conditional)
    # Only launched if localization_mode is 'amcl'
    localization_mode = LaunchConfiguration('localization_mode')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 2. Localization (AMCL + Map Server)
    # Explicitly launch nodes to ensure correct configuration and lifecycle management
    
    # Map Server
    map_server_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml_file}]
    )

    # AMCL
    amcl_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Lifecycle Manager for Localization
    lifecycle_manager_localization_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mapping', default_value='false', description='Enable map saving (PCD)'),
        DeclareLaunchArgument('localization_mode', default_value='amcl', description='Localization mode: icp or amcl'),
        DeclareLaunchArgument('map', default_value='', description='Path to map yaml for AMCL'),
        DeclareLaunchArgument('params_file', default_value='', description='Nav2 parameters file'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2 nodes'),
        
        robot_base_launch,
        livox_driver_node,
        fast_lio_node,
        linefit_node,
        pointcloud_to_laserscan_node,
        static_tf_odom_camera,
        static_tf_body_base,
        
        # IMU Complementary Filter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('imu_complementary_filter'), 'launch', 'complementary_filter.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        map_server_node,
        amcl_node,
        lifecycle_manager_localization_node
    ])
