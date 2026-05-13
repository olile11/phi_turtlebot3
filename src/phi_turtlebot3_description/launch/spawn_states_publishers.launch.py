#!/usr/bin/env python3

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_NAME = 'phi_turtlebot3_description'


def generate_launch_description():
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_prefix = LaunchConfiguration('frame_prefix')
    enable_jsp   = LaunchConfiguration('enable_jsp')

    urdf_path = os.path.join(
        get_package_share_directory(PKG_NAME),
        'urdf',
        f'turtlebot3_{turtlebot3_model}.urdf',
    )

    robot_description = xacro.process_file(
        urdf_path, mappings={'namespace': ''}
    ).toxml()

    spawn_robot = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': frame_prefix,
            }],
    )
    spawn_joint = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(enable_jsp),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('frame_prefix',default_value='',),
        DeclareLaunchArgument('enable_jsp', default_value='true'),
        spawn_robot,
        spawn_joint
    ])
