from launch import LaunchDescription
from launch_ros.actions import Node

# Fleet home positions: four corners of a 400 m × 400 m square, 20 m AGL
FLEET: dict = {
    1: ( 200.0,  200.0, 20.0),
    2: (-200.0,  200.0, 20.0),
    3: ( 200.0, -200.0, 20.0),
    4: (-200.0, -200.0, 20.0),
}


def generate_launch_description() -> LaunchDescription:
    nodes = [
        # ── Python nodes ──────────────────────────────────────────────────
        Node(
            package='acsdg_c2',
            executable='c2_engine_node',
            name='c2_engine_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_c2',
            executable='interceptor_manager_node',
            name='interceptor_manager_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='acsdg_c2',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'difficulty': 0.5}],
        ),
    ]

    # ── One C++ controller per interceptor ────────────────────────────────
    for iid, (hx, hy, hz) in FLEET.items():
        nodes.append(
            Node(
                package='acsdg_c2',
                executable='interceptor_controller_node',
                name=f'interceptor_controller_{iid}',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'interceptor_id': iid,
                    'home_x': hx,
                    'home_y': hy,
                    'home_z': hz,
                }],
            )
        )

    return LaunchDescription(nodes)
