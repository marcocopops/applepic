"""Orchard world + ApplePic robot + ros_gz bridge + localization (headless or with GUI).

`localization:=truth` (default): the pose comes from the ground-truth odometry.
`localization:=gps`: noisy GPS (gps_noise) fused with the wheel odometry (localizer EKF).
Either way the robot pose for the other nodes is /localization/odom and TF map -> base_footprint.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from applepic_bringup.launch_args import declare_localization_args


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
    seed = int(LaunchConfiguration('gps_seed').perform(context))
    if seed >= 0:
        gz_args = f'--seed {seed} {gz_args}'   # gz sensor noise (IMU gyro) reproducible
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
    localization = _localization_mode(context)
    localizer = Node(
        package='applepic_navigation',
        executable='localizer',
        parameters=[{'use_sim_time': True, 'mode': localization,
                     'initial_yaw': float(LaunchConfiguration('yaw').perform(context))}],
        output='screen',
    )
    nodes = [gz_sim, robot_state_publisher, spawn, bridge, localizer]
    if localization == 'truth':
        # odom -> base_footprint from the ground truth (in gps mode the localizer publishes it).
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            arguments=['/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/odom/tf', '/tf')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
    else:
        nodes.append(Node(
            package='applepic_navigation',
            executable='gps_noise',
            parameters=[{
                'use_sim_time': True,
                'stddev': float(LaunchConfiguration('gps_noise').perform(context)),
                'bias_stddev': float(LaunchConfiguration('gps_bias').perform(context)),
                'bias_tau': float(LaunchConfiguration('gps_bias_tau').perform(context)),
                'seed': int(LaunchConfiguration('gps_seed').perform(context)),
            }],
            output='screen',
        ))
    return nodes


def _localization_mode(context):
    mode = LaunchConfiguration('localization').perform(context).lower()
    if mode not in ('truth', 'gps'):
        raise ValueError(f'localization must be truth or gps, got {mode!r}')
    return mode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value='orchard.sdf'),
        # Spawn on the depot by default.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        *declare_localization_args(),
        OpaqueFunction(function=_setup),
    ])
