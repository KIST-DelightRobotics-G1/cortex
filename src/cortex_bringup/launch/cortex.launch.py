"""Standard robot bringup: real YOLO by default; llm_demo is the explicit stub."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.logging import launch_config
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cortex_bringup.runtime_config import detector_overrides


def _nodes(context, session_dir='', record_session=False):
    params = str(Path(LaunchConfiguration('params_file').perform(context)).expanduser())
    profile = LaunchConfiguration('detector_profile').perform(context)
    if profile:
        profile = str(Path(profile).expanduser())
    overrides = detector_overrides(
        params, profile, LaunchConfiguration('model').perform(context),
        LaunchConfiguration('device').perform(context))
    only = LaunchConfiguration('detector_only').perform(context).lower()
    if only not in ('true', 'false'):
        raise ValueError('detector_only must be true or false')
    llm_backend = LaunchConfiguration('llm_backend').perform(context)
    if llm_backend and llm_backend not in ('dummy', 'gemini', 'openai'):
        raise ValueError('llm_backend must be dummy, gemini or openai')
    graph = [('cortex_perception', 'detector_node')]
    if only == 'false':
        graph += [('cortex_perception', 'stt_node'), ('cortex_cognition', 'llm_node'),
                  ('cortex_cognition', 'orchestrator_node'), ('cortex_action', 'tts_node'),
                  ('cortex_action', 'speaker_node'), ('cortex_gui', 'gui_bridge_node')]
    nodes = []
    for package, executable in graph:
        layers = [params] + ([profile] if profile else [])
        if executable == 'detector_node' and overrides:
            layers.append({key: ParameterValue(value, value_type=str)
                           for key, value in overrides.items()})
        if executable == 'llm_node' and llm_backend:
            layers.append({'backend': ParameterValue(llm_backend, value_type=str)})
        if record_session and executable == 'detector_node':
            layers.append({'diagnostics_enabled': True})
        if record_session and executable == 'orchestrator_node':
            layers.append({'detector_diagnostics_enabled': True})
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output='both', parameters=layers,
                          additional_env={'ROS_LOG_DIR': session_dir} if session_dir else {}))
    return nodes


def _launch_setup(context):
    record = LaunchConfiguration('record_session').perform(context).lower()
    if record not in ('true', 'false'):
        raise ValueError('record_session must be true or false')
    # Reuse the ROS launch run directory, including its launch.log file handler.
    session_dir = launch_config.log_dir
    nodes = _nodes(context, session_dir, record == 'true')
    actions = [LogInfo(msg=f'Cortex session logs: {session_dir}')]
    if record == 'true':
        share = Path(get_package_share_directory('cortex_bringup'))
        cmd = [sys.executable, str(share/'scripts/collect_yolo_diagnostics.py'),
               '--output', str(Path(session_dir)/'diagnostics'), '--duration', '0',
               '--wait-for-nodes', '45']
        if LaunchConfiguration('detector_only').perform(context).lower() == 'true':
            cmd += ['--orchestrator-node', '']
        actions.append(ExecuteProcess(
            cmd=cmd, name='session_recorder', output='both',
            additional_env={'ROS_LOG_DIR': session_dir},
            sigterm_timeout='60', sigkill_timeout='5'))
    # Recording observes the graph; it never gates commands or stops other nodes.
    return actions + nodes


def generate_launch_description():
    share = Path(get_package_share_directory('cortex_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/cortex_params.yaml'),
                              description='Full ROS parameter YAML (site configuration)'),
        DeclareLaunchArgument('detector_profile', default_value='',
                              description='Optional parameter YAML overlay, absolute path'),
        DeclareLaunchArgument('model', default_value='',
                              description='Override detector_node.model; empty keeps YAML value'),
        DeclareLaunchArgument('device', default_value='',
                              description='Override YAML device, e.g. 0, cpu or mps'),
        DeclareLaunchArgument('detector_only', default_value='false'),
        DeclareLaunchArgument('llm_backend', default_value='',
                              description='Override llm_node.backend: dummy | gemini | openai; '
                                          'empty keeps YAML value'),
        DeclareLaunchArgument('record_session', default_value='true',
                              description='Record camera, execution and YOLO diagnostics until shutdown; '
                                          'false keeps text logs only'),
        OpaqueFunction(function=_launch_setup),
    ])
