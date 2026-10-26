"""Simulation (sim.launch.py) + waypoint follower doing the orchard loop."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('applepic_bringup'), 'launch', 'sim.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )
    follower = Node(
        package='applepic_navigation',
        executable='waypoint_follower',
        parameters=[{
            'use_sim_time': True,
            'loops': LaunchConfiguration('loops'),
            'trajectory_csv': LaunchConfiguration('trajectory_csv'),
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('loops', default_value='0'),
        DeclareLaunchArgument('trajectory_csv', default_value=''),
        sim,
        # Give gz sim time to start and spawn the robot before driving.
        TimerAction(period=8.0, actions=[follower]),
    ])
