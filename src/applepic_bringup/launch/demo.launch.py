"""Demo (M8): full mission with the Gazebo GUI, RViz (orchard, trajectory, camera, HUD).

Args: headless (default false), rviz (default true), rtf (real-time factor, default 1),
plus the mission.launch.py args (x, y, yaw, capacity, loops, search_radius).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MISSION_ARGS = ('headless', 'x', 'y', 'yaw', 'capacity', 'loops', 'search_radius')

# Retry until the world is up; the GUI can rarely keep up beyond ~2-3x.
SET_RTF = (
    'for i in $(seq 60); do '
    'gz service -s /world/orchard/set_physics --reqtype gz.msgs.Physics '
    '--reptype gz.msgs.Boolean --timeout 2000 '
    '--req "max_step_size: 0.001, real_time_factor: $0" 2>/dev/null | grep -q true '
    '&& echo "real-time factor set to $0" && exit 0; sleep 1; done; '
    'echo "could not set real-time factor" >&2'
)


def generate_launch_description():
    share = get_package_share_directory('applepic_bringup')
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mission.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in MISSION_ARGS}.items(),
    )
    viz = Node(
        package='applepic_mission',
        executable='mission_viz',
        parameters=[{
            'use_sim_time': True,
            'capacity': ParameterValue(LaunchConfiguration('capacity'), value_type=int),
            'search_radius': ParameterValue(LaunchConfiguration('search_radius'),
                                            value_type=float),
        }],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'demo.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )
    set_rtf = ExecuteProcess(
        cmd=['bash', '-c', SET_RTF, LaunchConfiguration('rtf')],
        condition=LaunchConfigurationNotEquals('rtf', '1'),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtf', default_value='1', description='simulation speed-up'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('capacity', default_value='10'),
        DeclareLaunchArgument('loops', default_value='1', description='0 = forever'),
        DeclareLaunchArgument('search_radius', default_value='2.2'),
        mission,
        viz,
        rviz,
        set_rtf,
    ])
