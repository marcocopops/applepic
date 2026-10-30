"""Localization launch arguments shared by the launch files (declared by sim.launch.py)."""
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

LOCALIZATION_ARGS = (
    ('localization', 'truth', 'truth (ground-truth pose) or gps (GPS + wheel odometry EKF)'),
    ('gps_noise', '0.02', 'm, GPS white noise std (0.02 = RTK grade)'),
    ('gps_bias', '0.0', 'm, GPS wandering bias std'),
    ('gps_bias_tau', '60.0', 's, GPS bias correlation time'),
    ('gps_seed', '0', 'seed of the GPS error and of the gz sensor noise, < 0: random'),
)


def declare_localization_args():
    return [DeclareLaunchArgument(n, default_value=d, description=t)
            for n, d, t in LOCALIZATION_ARGS]


def localization_configs():
    """{name: LaunchConfiguration} to forward to an included launch file."""
    return {n: LaunchConfiguration(n) for n, _, _ in LOCALIZATION_ARGS}
