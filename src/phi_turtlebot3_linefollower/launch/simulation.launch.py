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

# burger_cam é a variante com câmera (320x240). Setamos antes do
# generate_launch_description porque os spawn launches da description
# leem TURTLEBOT3_MODEL via os.environ no momento em que geram sua
# launch description (mesmo processo Python).
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

    sdf_path = os.path.join(
        description_share, 'models', ROBOT_NAME, 'model.sdf')
    bridge_config = os.path.join(
        linefollower_share, 'config', 'model_bridge.yaml')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    states_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_states_publishers.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Spawn manual — não usamos spawn_turtlebot3.launch.py porque ele
    # sobe um parameter_bridge inline com seu próprio YAML. Aqui o
    # bridge é centralizado em config/model_bridge.yaml.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ROBOT_NAME,
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    )

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch, 'spawn_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    line_follower_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(linefollower_launch, 'line_follower.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Viewer cv2: raw da câmera (esq) + debug com centróide (dir) em uma
    # janela só. cv2.imshow/waitKey rodam na main thread deste processo
    # — diferente do line_follower, onde travavam o executor no shutdown.
    image_view_cmd = Node(
        package='phi_turtlebot3_linefollower',
        executable='viewer',
        output='screen',
    )

    return [gazebo_cmd, states_cmd, spawn_robot, bridge_cmd, rviz_cmd,
            line_follower_cmd, image_view_cmd]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='line_world',
            description='World name (resolvido em phi_turtlebot3_description/worlds).',
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
