#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG_NAME = 'phi_turtlebot3_description'

WORLD_ALIASES = {
    'empty': 'empty_world',
    'house': 'turtlebot3_house',
}

WORLD_EXTENSIONS = ('.sdf', '.world')


def resolve_world_path(world):
    if os.path.isabs(world):
        return world

    world_name = WORLD_ALIASES.get(world, world)
    worlds_dir = os.path.join(
        get_package_share_directory(PKG_NAME), 'worlds')

    if os.path.splitext(world_name)[1] in WORLD_EXTENSIONS:
        return os.path.join(worlds_dir, world_name)

    for ext in WORLD_EXTENSIONS:
        candidate = os.path.join(worlds_dir, world_name + ext)
        if os.path.exists(candidate):
            return candidate

    # Fallback: deixa o Gazebo reclamar com nome original.
    return os.path.join(worlds_dir, world_name + WORLD_EXTENSIONS[0])


def launch_gazebo(context, *args, **kwargs):
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = resolve_world_path(LaunchConfiguration('world').perform(context))
    verbosity = LaunchConfiguration('gz_verbosity').perform(context)

    spawn_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v{verbosity} {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-g -v{verbosity}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return [spawn_gzserver, spawn_gzclient]


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG_NAME)

    return LaunchDescription([
        DeclareLaunchArgument('world',default_value='obstacles'),
        DeclareLaunchArgument('gz_verbosity',default_value='2'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.path.join(pkg_share, 'models'),),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.path.join(pkg_share, 'worlds'),),
        OpaqueFunction(function=launch_gazebo),
    ])
