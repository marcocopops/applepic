"""Full mission: simulation + apple detector + magic gripper + mission state machine (M6)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from applepic_bringup.launch_args import declare_localization_args, localization_configs


def generate_launch_description():
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('applepic_bringup'), 'launch', 'perception.launch.py')),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in ('headless', 'world', 'x', 'y', 'yaw')},
            **localization_configs(),
        }.items(),
    )
    gripper = Node(
        package='applepic_mission',
        executable='gripper',
        parameters=[{'use_sim_time': True,
                     'capacity': ParameterValue(LaunchConfiguration('capacity'), value_type=int)}],
        output='screen',
    )
    mission = Node(
        package='applepic_mission',
        executable='mission',
        parameters=[{
            'use_sim_time': True,
            'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int),
            'search_radius': ParameterValue(LaunchConfiguration('search_radius'),
                                            value_type=float),
            'trajectory_csv': LaunchConfiguration('trajectory_csv'),
            'exit_when_done': ParameterValue(LaunchConfiguration('exit_when_done'),
                                             value_type=bool),
            # Only a coarse GPS has an error worth correcting on the trunks (RTK does not).
            'anchor': ParameterValue(PythonExpression(
                ["'", LaunchConfiguration('localization'), "' == 'gps' and max(",
                 LaunchConfiguration('gps_noise'), ', ', LaunchConfiguration('gps_bias'),
                 ') > 0.1']), value_type=bool),
        }],
        output='screen',
    )
    return LaunchDescription([
        *declare_localization_args(),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value='orchard.sdf',
                              description='SDF file name in applepic_gazebo/worlds or absolute path'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('capacity', default_value='10'),
        DeclareLaunchArgument('loops', default_value='1', description='0 = forever'),
        DeclareLaunchArgument('search_radius', default_value='1.9'),
        DeclareLaunchArgument('trajectory_csv', default_value=''),
        DeclareLaunchArgument('exit_when_done', default_value='false'),
        perception,
        gripper,
        mission,
    ])
