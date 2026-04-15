from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='acsdg_ai',
            executable='threat_predictor_node',
            name='threat_predictor_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_ai',
            executable='swarm_classifier_node',
            name='swarm_classifier_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_ai',
            executable='learning_node',
            name='learning_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
