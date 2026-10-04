"""Compatibility entry point; new deployments use cortex.launch.py."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(context):
    share = Path(get_package_share_directory('cortex_bringup'))
    profile = Path(LaunchConfiguration('profile').perform(context)).expanduser()
    if not profile.is_absolute():
        profile = share/'config'/profile
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch/cortex.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file').perform(context),
            'detector_profile': str(profile),
            'model': LaunchConfiguration('model').perform(context),
            'device': LaunchConfiguration('device').perform(context),
            'detector_only': LaunchConfiguration('detector_only').perform(context),
        }.items())]


def generate_launch_description():
    share = Path(get_package_share_directory('cortex_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/cortex_params.yaml')),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('profile', default_value='fridge_detector_v3.yaml'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('detector_only', default_value='false'),
        OpaqueFunction(function=_include),
    ])
