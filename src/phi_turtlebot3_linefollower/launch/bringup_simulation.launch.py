#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

DESCRIPTION_PKG = 'phi_turtlebot3_description'
LINEFOLLOWER_PKG = 'phi_turtlebot3_linefollower'

os.environ['TURTLEBOT3_MODEL'] = 'burger_cam'

ROBOT_NAME = 'turtlebot3_burger_cam'


def launch_setup(context, *args, **kwargs):
    description_share = get_package_share_directory(DESCRIPTION_PKG)
    linefollower_share = get_package_share_directory(LINEFOLLOWER_PKG)

    description_launch = os.path.join(description_share, 'launch')
    linefollower_launch = os.path.join(linefollower_share, 'launch')

    world = context.launch_configurations['world']
    use_sim_time = context.launch_configurations['use_sim_time']
    x_pose = context.launch_configurations['x_pose']
    y_pose = context.launch_configurations['y_pose']

    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    spawn_states_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_states_publishers.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose, 'z_yaw': '-1.35'}.items(),
    )

    spawn_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(linefollower_launch, 'bringup_line_follower.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_viewer = Node(
        package='phi_turtlebot3_linefollower',
        executable='viewer',
        output='screen',
    )

    return [spawn_gazebo, 
            spawn_states_publishers,
            spawn_model, 
            spawn_rviz, 
            spawn_controller, 
            spawn_viewer]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world',default_value='line_world'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
