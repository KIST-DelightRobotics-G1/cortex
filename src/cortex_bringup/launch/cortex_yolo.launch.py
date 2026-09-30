"""Real local YOLO weight profile; default demo launch remains unchanged."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = Path(get_package_share_directory('cortex_bringup'))
    model = Path(LaunchConfiguration('model').perform(context)).expanduser()
    if not model.is_file():
        raise ValueError(f'model must name an existing local .pt file: {model}')
    profile_name = LaunchConfiguration('profile').perform(context)
    if profile_name not in ('fridge_detector.yaml', 'fridge_detector_v3.yaml'):
        raise ValueError('unsupported detector profile')
    profile = share/'config'/profile_name
    if not profile.is_file():
        raise ValueError('profile must be fridge_detector.yaml or fridge_detector_v3.yaml')
    device = LaunchConfiguration('device').perform(context)
    only = LaunchConfiguration('detector_only').perform(context).lower()
    if only not in ('true', 'false'):
        raise ValueError('detector_only must be true or false')
    graph = [('cortex_perception', 'detector_node')]
    if only == 'false':
        graph += [('cortex_perception', 'stt_node'), ('cortex_cognition', 'llm_node'),
                  ('cortex_cognition', 'orchestrator_node'), ('cortex_action', 'tts_node'),
                  ('cortex_gui', 'gui_bridge_node')]
    nodes = []
    for package, executable in graph:
        params = [str(share/'config/cortex_params.yaml'), str(profile)]
        if executable == 'detector_node':
            params.append({'model': str(model.resolve()), 'device': device})
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output='screen', parameters=params))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', description='Existing local YOLO detection .pt weight'),
        DeclareLaunchArgument('profile', default_value='fridge_detector_v3.yaml',
                              description='Versioned real-detector configuration'),
        DeclareLaunchArgument('device', default_value='', description='Ultralytics device, e.g. cpu, 0, mps'),
        DeclareLaunchArgument('detector_only', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
