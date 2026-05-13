#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_description_name = 'phi_turtlebot3_description'
pkg_exploration_name = 'phi_turtlebot3_exploration'

WORLD_EXTENSIONS = ('.world', '.sdf')


def _resolve_world_filename(world_val: str, worlds_dir: str) -> str:
    if os.path.splitext(world_val)[1]:
        return world_val
    for ext in WORLD_EXTENSIONS:
        if os.path.exists(os.path.join(worlds_dir, world_val + ext)):
            return world_val + ext
    return world_val + WORLD_EXTENSIONS[0]


def launch_setup(context, *args, **kwargs):
    description_launch_dir = os.path.join(
        get_package_share_directory(pkg_description_name), 'launch'
    )
    exploration_launch_dir = os.path.join(
        get_package_share_directory(pkg_exploration_name), 'launch'
    )
    worlds_dir = os.path.join(
        get_package_share_directory(pkg_description_name), 'worlds'
    )

    world_filename = _resolve_world_filename(
        context.launch_configurations['world'], worlds_dir
    )

    use_sim_time = context.launch_configurations['use_sim_time']
    x_pose = context.launch_configurations['x_pose']
    y_pose = context.launch_configurations['y_pose']

    spawn_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_gazebo.launch.py')
        ),
        launch_arguments={'world': world_filename}.items(),
    )

    spawn_states_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_states_publishers.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    spawn_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    exploration_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_launch_dir, 'bringup_exploration.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return [
        spawn_gazebo, 
        spawn_states_publishers, 
        spawn_model, 
        spawn_slam, 
        spawn_rviz, 
        exploration_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='obstacles'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
