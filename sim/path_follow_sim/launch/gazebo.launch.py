"""Visible Gazebo entry point; retain the world after the test completes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='gazebo'),
        DeclareLaunchArgument('exit_on_finish', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('path_follow_sim'), 'launch', 'sim.launch.py')))
    ])
