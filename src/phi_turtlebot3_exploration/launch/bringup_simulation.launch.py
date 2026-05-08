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
    """Return world_val with an extension, probing .world then .sdf if none given."""
    if os.path.splitext(world_val)[1]:
        return world_val
    for ext in WORLD_EXTENSIONS:
        if os.path.exists(os.path.join(worlds_dir, world_val + ext)):
            return world_val + ext
    return world_val + WORLD_EXTENSIONS[0]


def launch_setup(context, *args, **kwargs):
    pkg_description_dir = os.path.join(
        get_package_share_directory(pkg_description_name), 'launch'
    )
    pkg_exploration_dir = os.path.join(
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

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description_dir, 'spawn_gazebo.launch.py')
        ),
        launch_arguments={'world': world_filename}.items(),
    )

    states_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description_dir, 'spawn_states_publishers.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    turtlebot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description_dir, 'spawn_slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description_dir, 'spawn_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    exploration_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_exploration_dir, 'bringup_exploration.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return [gazebo_node, states_nodes, slam_node, turtlebot_node, rviz_node, exploration_node]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='obstacles',
        description=(
            'World/SDF file name with or without extension '
            '(e.g. obstacles, turtlebot3_world, house.sdf).'
        ),
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial X position of the robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial Y position of the robot'
    )

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    return LaunchDescription([
        world_arg,
        x_pose_arg,
        y_pose_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup),
    ])
