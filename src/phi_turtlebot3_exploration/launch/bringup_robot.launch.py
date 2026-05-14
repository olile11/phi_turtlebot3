#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

DESCRIPTION_PKG = 'phi_turtlebot3_description'
EXPLORATION_PKG = 'phi_turtlebot3_exploration'

ROS_DOMAIN_ID = '30'

def generate_launch_description():
    description_launch = os.path.join(
        get_package_share_directory(DESCRIPTION_PKG), 'launch'
    )
    exploration_launch = os.path.join(
        get_package_share_directory(EXPLORATION_PKG), 'launch'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz    = LaunchConfiguration('open_rviz')
    open_slam    = LaunchConfiguration('open_slam')

    spawn_states_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch,
                         'spawn_states_publishers.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_jsp': 'false',
        }.items(),
    )

    spawn_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(open_slam),
    )

    spawn_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(open_rviz),
    )

    spawn_exploration = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exploration_launch, 'bringup_exploration.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', ROS_DOMAIN_ID),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('open_rviz', default_value='false'),
        DeclareLaunchArgument('open_slam', default_value='false'),
        spawn_states_publishers,
        spawn_slam,
        spawn_rviz,
        spawn_exploration,
    ])
