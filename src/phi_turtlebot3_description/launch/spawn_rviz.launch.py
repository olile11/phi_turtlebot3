#!/usr/bin/env python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_NAME = 'phi_turtlebot3_description'


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG_NAME)

    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    spawn_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=join(pkg_share, 'rviz', 'model.rviz')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        spawn_rviz_node,
    ])
