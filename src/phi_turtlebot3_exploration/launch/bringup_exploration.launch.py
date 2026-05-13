#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_stamped = LaunchConfiguration('cmd_vel_stamped')

    exploration_node = Node(
        package='phi_turtlebot3_exploration',
        executable="potential_field",
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_stamped': cmd_vel_stamped,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock.'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='true'),
        exploration_node,
    ])
