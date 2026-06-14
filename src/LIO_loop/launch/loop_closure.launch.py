import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('LIO_loop')
    config_path = os.path.join(pkg_dir, 'config', 'loop_closure.yaml')

    loop_closure_node = Node(
        package='LIO_loop',
        executable='loop_closure_node',
        parameters=[config_path],
        output='screen',
        remappings=[
            ('/Odometry', '/Odometry'),
            ('/cloud_registered', '/cloud_registered'),
        ],
    )

    return LaunchDescription([loop_closure_node])
