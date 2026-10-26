"""Simulation (sim.launch.py) + HSV apple detector publishing /apples/detections."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('applepic_bringup'), 'launch', 'sim.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name) for name in ('headless', 'x', 'y', 'yaw')
        }.items(),
    )
    detector = Node(
        package='applepic_perception',
        executable='apple_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        sim,
        detector,
    ])
