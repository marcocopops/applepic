"""Simulation + apple detector + magic gripper + apple picker (M5: pick the visible apples)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('applepic_bringup'), 'launch', 'perception.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name) for name in ('headless', 'x', 'y', 'yaw')
        }.items(),
    )
    gripper = Node(
        package='applepic_mission',
        executable='gripper',
        parameters=[{'use_sim_time': True,
                     'capacity': LaunchConfiguration('capacity')}],
        output='screen',
    )
    picker = Node(
        package='applepic_mission',
        executable='apple_picker',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('x', default_value='2.6'),
        DeclareLaunchArgument('y', default_value='4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('capacity', default_value='10'),
        perception,
        gripper,
        picker,
    ])
