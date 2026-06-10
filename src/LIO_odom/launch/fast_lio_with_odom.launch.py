from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lio_odom_share = get_package_share_directory('lio_odom')
    fast_lio_share = get_package_share_directory('fast_lio')

    transformer_config = os.path.join(lio_odom_share, 'config', 'odom_transformer.yaml')
    fast_lio_launch = os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
    fast_lio_config_path = os.path.join(lio_odom_share, 'config')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                'config_path': fast_lio_config_path,
                'config_file': 'fast_lio_minimal.yaml',
                'rviz': 'false'
            }.items()
        ),
        Node(
            package='lio_odom',
            executable='odom_transformer',
            name='lio_odom_transformer',
            output='screen',
            parameters=[transformer_config]
        )
    ])
