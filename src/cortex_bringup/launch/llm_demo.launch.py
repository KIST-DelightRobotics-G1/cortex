"""Offline llm-mode loop: no robot, no mic, no network.

    llm_node(dummy) -> orchestrator_node(llm) -> mock nav + mock vla -> gui_bridge

Drive it from a terminal:
    ros2 topic pub -1 /cortex/stt/transcript std_msgs/msg/String "{data: '냉장고에서 오이 가져다줘'}"
    ros2 topic echo /cortex/trace
Then open kist-drl-g1-gui at ws://localhost:8081.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory('cortex_bringup'), 'config', 'cortex_params.yaml')

    def node(pkg, exe, name=None, extra=None):
        return Node(package=pkg, executable=exe, name=name or exe, output='screen',
                    parameters=[params] + ([extra] if extra else []))

    return LaunchDescription([
        # backend:=gemini needs GOOGLE_API_KEY in the environment (openai: OPENAI_API_KEY)
        DeclareLaunchArgument('backend', default_value='dummy', description='dummy | gemini | openai'),
        node('cortex_cognition', 'llm_node', extra={'backend': LaunchConfiguration('backend')}),
        node('cortex_cognition', 'orchestrator_node', extra={'planner_mode': 'llm'}),
        node('cortex_cognition', 'mock_module_node', 'mock_nav',
             {'role': 'nav', 'cmd_topic': '/cortex/nav/cmd', 'state_topic': '/cortex/nav/state',
              'duration_s': 3.0}),
        node('cortex_cognition', 'mock_module_node', 'mock_vla',
             {'role': 'vla', 'cmd_topic': '/cortex/vla/cmd', 'state_topic': '/cortex/vla/state',
              'duration_s': 2.0}),
        node('cortex_gui', 'gui_bridge_node', extra={'camera_transport': 'none'}),
    ])
