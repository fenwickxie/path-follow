"""Publish a standard nav_msgs/Path for Pure Pursuit or another path follower.

The reliable transient-local publisher retains the latest path for subscribers
that start after this publisher, without repeatedly resetting their path state.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('path_follow_sim')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s_curve'),
        DeclareLaunchArgument('path_topic', default_value='/plan'),
        DeclareLaunchArgument('frame', default_value='odom'),
        DeclareLaunchArgument('spacing', default_value='0.05'),
        DeclareLaunchArgument('start_delay', default_value='1.0'),
        DeclareLaunchArgument('keep_heading', default_value='false'),
        DeclareLaunchArgument('heading', default_value='0.0'),
        DeclareLaunchArgument('x0', default_value='0.0'),
        DeclareLaunchArgument('y0', default_value='0.0'),
        DeclareLaunchArgument('length', default_value='8.0'),
        DeclareLaunchArgument('radius', default_value='3.0'),
        DeclareLaunchArgument('arc_angle', default_value='1.5708'),
        DeclareLaunchArgument('amplitude', default_value='1.0'),
        Node(
            package='path_follow_sim',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
            parameters=[os.path.join(share, 'config', 'sim.yaml'), {
                'topic': LaunchConfiguration('path_topic'),
                'frame': LaunchConfiguration('frame'),
                'scenario': LaunchConfiguration('scenario'),
                'spacing': LaunchConfiguration('spacing'),
                'start_delay': LaunchConfiguration('start_delay'),
                'keep_heading': LaunchConfiguration('keep_heading'),
                'heading': LaunchConfiguration('heading'),
                'params.x0': LaunchConfiguration('x0'),
                'params.y0': LaunchConfiguration('y0'),
                'params.length': LaunchConfiguration('length'),
                'params.radius': LaunchConfiguration('radius'),
                'params.arc_angle': LaunchConfiguration('arc_angle'),
                'params.amplitude': LaunchConfiguration('amplitude'),
                'transient_local': True,
                'wait_for_subscriber': True,
            }]),
    ])
