from launch import LaunchDescription
from launch_ros.actions import Node

# Fleet home positions: four corners of a 400 m × 400 m square, 20 m AGL.
# Phase 2: slot 1 (NE) is a Coyote Block 2; slots 2/3/4 remain Anvil drones.
FLEET: dict = {
    1: ( 200.0,  200.0, 20.0),    # NE — Coyote Block 2 (Phase 2)
    2: (-200.0,  200.0, 20.0),    # NW — Anvil
    3: ( 200.0, -200.0, 20.0),    # SE — Anvil
    4: (-200.0, -200.0, 20.0),    # SW — Anvil
}

# Slot id whose controller is the Coyote (jet, no rotors). All other slots
# spawn the legacy Anvil interceptor_controller_node.
COYOTE_SLOT = 1


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

    # ── One C++ controller per slot (Coyote at COYOTE_SLOT, Anvil elsewhere) ─
    for iid, (hx, hy, hz) in FLEET.items():
        executable = ('coyote_controller_node' if iid == COYOTE_SLOT
                      else 'interceptor_controller_node')
        node_name = (f'coyote_controller_{iid}' if iid == COYOTE_SLOT
                     else f'interceptor_controller_{iid}')
        nodes.append(
            Node(
                package='acsdg_c2',
                executable=executable,
                name=node_name,
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
