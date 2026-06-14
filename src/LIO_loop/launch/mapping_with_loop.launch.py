import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ---- FAST-LIO shared dirs ----
    fast_lio_pkg = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(fast_lio_pkg, 'config')
    default_rviz_config_path = os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')

    # ---- Loop closure shared dirs ----
    loop_pkg = get_package_share_directory('LIO_loop')
    default_loop_config = os.path.join(loop_pkg, 'config', 'loop_closure.yaml')

    # ---- Launch arguments ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    loop_use = LaunchConfiguration('loop')
    loop_config = LaunchConfiguration('loop_config')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='FAST-LIO YAML config directory'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='FAST-LIO config file name'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_loop_cmd = DeclareLaunchArgument(
        'loop', default_value='true',
        description='Enable loop closure module'
    )
    declare_loop_config_cmd = DeclareLaunchArgument(
        'loop_config', default_value=default_loop_config,
        description='Loop closure YAML config path'
    )

    # ---- Nodes ----
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    loop_closure_node = Node(
        package='LIO_loop',
        executable='loop_closure_node',
        parameters=[loop_config],
        output='screen',
        condition=IfCondition(loop_use),
        remappings=[
            ('/Odometry', '/Odometry'),
            ('/cloud_registered', '/cloud_registered'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # ---- Assemble ----
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_loop_cmd)
    ld.add_action(declare_loop_config_cmd)
    ld.add_action(fast_lio_node)
    ld.add_action(loop_closure_node)
    ld.add_action(rviz_node)

    return ld
