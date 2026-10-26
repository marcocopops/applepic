"""Orchard world + ApplePic robot + ros_gz bridge (headless or with GUI)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _setup(context):
    gazebo_share = get_package_share_directory('applepic_gazebo')
    description_share = get_package_share_directory('applepic_description')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(gazebo_share, 'worlds', world)

    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'applepic.urdf.xacro')).toxml()

    gz_args = f'-r {world}'
    if headless:
        gz_args = f'-s --headless-rendering {gz_args}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'applepic',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x').perform(context),
            '-y', LaunchConfiguration('y').perform(context),
            '-z', '0.02',
            '-Y', LaunchConfiguration('yaw').perform(context),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )
    return [gz_sim, robot_state_publisher, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value='orchard.sdf'),
        # Spawn on the depot by default.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
