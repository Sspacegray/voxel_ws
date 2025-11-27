from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                    {'publish_tf': False},  # 禁用TF发布，避免与Fast-LIO冲突
                ],
                remappings=[
                	('/imu/data_raw', '/livox/imu'),
                ]
            )
        ]
    )
