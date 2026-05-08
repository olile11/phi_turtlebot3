#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration_executable = LaunchConfiguration('exploration_executable')
    cmd_vel_stamped = LaunchConfiguration('cmd_vel_stamped')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock.',
    )

    declare_exploration_executable_cmd = DeclareLaunchArgument(
        'exploration_executable',
        default_value='potential_field',
        description='Exploration node executable.',
    )

    declare_cmd_vel_stamped_cmd = DeclareLaunchArgument(
        'cmd_vel_stamped',
        default_value='true',
        description='Publica TwistStamped (sim) ou Twist (TB3 real).',
    )

    exploration_node = Node(
        package='phi_turtlebot3_exploration',
        executable=exploration_executable,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_stamped': cmd_vel_stamped,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_exploration_executable_cmd)
    ld.add_action(declare_cmd_vel_stamped_cmd)
    ld.add_action(exploration_node)

    return ld
