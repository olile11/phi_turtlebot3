from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    description_pkg = get_package_share_directory('phi_turtlebot3_description')

    slam_params_file = join(description_pkg, 'config', 'slam_config.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='true em sim (Gazebo); false no robô real.',
        ),
        slam_launch,
    ])
