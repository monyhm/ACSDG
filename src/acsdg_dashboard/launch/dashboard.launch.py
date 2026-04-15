from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ── rosbridge WebSocket server on port 9090 ───────────────────────
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0.0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0,
            }],
        ),

        # ── React app HTTP server on port 3000 ───────────────────────────
        Node(
            package='acsdg_dashboard',
            executable='dashboard_server_node',
            name='dashboard_server_node',
            output='screen',
            emulate_tty=True,
        ),

        LogInfo(msg='─────────────────────────────────────────────────'),
        LogInfo(msg='  ACSDG Dashboard: http://localhost:3000'),
        LogInfo(msg='  rosbridge WebSocket: ws://localhost:9090'),
        LogInfo(msg='─────────────────────────────────────────────────'),
    ])
