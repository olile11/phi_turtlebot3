#!/usr/bin/env python3
"""
Bringup do TurtleBot3 real (sem Gazebo).

Pré-requisitos:
  • Robô e PC remoto na mesma Wi-Fi (SSID: turtlebot3, senha: turtlebot3).
  • IPs dos robôs no roteador 192.168.0.1 (admin/admin):
      Turtlebot1 → 192.168.0.100
      Turtlebot2 → 192.168.0.101
  • No SBC do robô (após `ssh ubuntu@<IP>`, senha turtlebot):
        export ROS_DOMAIN_ID=30
        export TURTLEBOT3_MODEL=burger
        ros2 launch turtlebot3_bringup robot.launch.py

No PC remoto:
        export TURTLEBOT3_MODEL=burger
        ros2 launch phi_turtlebot3_exploration bringup_robot.launch.py

Este launch sobe SLAM (slam_toolbox), RViz, robot_state_publisher e o
explorador (potential_field). Subimos o robot_state_publisher localmente
com a nossa URDF porque o /robot_description publicado pelo SBC pode vir
com xacro não processado (links com `${namespace}` literal). Como
/robot_description é TRANSIENT_LOCAL+depth=1, o publisher mais recente
vence — o RViz acaba lendo a nossa.
"""

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
    description_launch_dir = os.path.join(
        get_package_share_directory(DESCRIPTION_PKG), 'launch'
    )
    exploration_launch_dir = os.path.join(
        get_package_share_directory(EXPLORATION_PKG), 'launch'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz    = LaunchConfiguration('open_rviz')
    open_slam    = LaunchConfiguration('open_slam')

    states_publishers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir,
                         'spawn_states_publishers.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_jsp': 'false',
        }.items(),
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(open_slam),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'spawn_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(open_rviz),
    )

    # Atrasa o explorador para o slam_toolbox processar a primeira scan
    # e publicar map → odom antes do TF lookup. Sem isso, o explorador
    # cospe TransformException nos primeiros segundos.
    exploration_cmd = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exploration_launch_dir, 'bringup_exploration.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', ROS_DOMAIN_ID),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Robô real → false. Sim → true.',
        ),
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Abrir RViz no PC remoto.',
        ),
        DeclareLaunchArgument(
            'open_slam',
            default_value='false',
            description='Subir slam_toolbox no PC remoto. Desligue se já '
                        'estiver rodando SLAM por fora.',
        ),
        states_publishers_cmd,
        slam_cmd,
        rviz_cmd,
        exploration_cmd,
    ])
