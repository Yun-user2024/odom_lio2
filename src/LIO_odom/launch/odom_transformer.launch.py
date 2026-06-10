from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lio_odom'),
        'config',
        'odom_transformer.yaml'
    )

    return LaunchDescription([
        Node(
            package='lio_odom',
            executable='odom_transformer',
            name='lio_odom_transformer',
            output='screen',
            parameters=[config_file]
        )
    ])
