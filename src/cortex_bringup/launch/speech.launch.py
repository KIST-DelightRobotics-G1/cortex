"""Speech I/O only: STT (perception) + TTS and the robot speaker (action).

A subset of cortex.launch.py for exercising the audio path without the cognition
layer. The two nodes live in different packages now — stt_node is perception
(audio -> symbols), tts_node is action (the robot speaking).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cortex_bringup.runtime_config import node_logging


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(get_package_share_directory('cortex_bringup'), 'config')
    params = os.path.join(config, 'cortex_params.yaml')

    def node(pkg, exe):
        level, events, record = node_logging(os.path.join(config, 'logging.yaml'), exe)
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=[params, {'log_events': ParameterValue(events, value_type=str),
                                         'record_events': ParameterValue(record, value_type=str)}],
                    arguments=['--ros-args', '--log-level', level])

    return LaunchDescription([
        node('cortex_perception', 'stt_node'),
        node('cortex_action', 'tts_node'),
        node('cortex_action', 'speaker_node'),
    ])
