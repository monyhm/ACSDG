"""ACSDG C2 launch — spawns the C2 engine, mission manager, interceptor manager,
and one controller per FLEET slot.

Per-slot weapon selection comes from acsdg_c2.fleet.FLEET — single source of
truth for which weapon class occupies which post. Adding a weapon (Phase 3
DroneHunter, Phase 4 Skyranger) means appending one Slot to FLEET. No code
change here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from acsdg_c2.fleet import FLEET
from acsdg_c2.weapons.types import TargetClass


def generate_launch_description() -> LaunchDescription:
    nodes: list[Node] = [
        Node(package='acsdg_c2', executable='c2_engine_node',
             name='c2_engine_node', output='screen', emulate_tty=True),
        Node(package='acsdg_c2', executable='interceptor_manager_node',
             name='interceptor_manager_node', output='screen', emulate_tty=True),
        Node(package='acsdg_c2', executable='mission_manager_node',
             name='mission_manager_node', output='screen', emulate_tty=True,
             parameters=[{'difficulty': 0.5}]),
    ]

    for slot in FLEET:
        # Build a per-slot template so we can pull pkill values for parameters.
        # The template instance is throw-away; the real weapon objects live in
        # c2_engine_node._weapons (also built from FLEET).
        template = slot.weapon_class(weapon_id=slot.weapon_id, home_position=slot.home)
        parameters: list[dict] = [
            {'interceptor_id': slot.interceptor_id},
            {'home_x': slot.home[0]},
            {'home_y': slot.home[1]},
            {'home_z': slot.home[2]},
        ]
        # Coyote needs frag-fuze pkill + RNG seed parameters. Other weapons may
        # add their own per-class parameters here in future phases.
        if slot.weapon_class.__name__ == 'Coyote':
            parameters.append({'pkill_small_quad': template.pkill(TargetClass.SMALL_QUAD)})
            parameters.append({'rng_seed': -1})  # production: cryptographic seed

        nodes.append(Node(
            package='acsdg_c2',
            executable=slot.controller_executable,
            name=f"{slot.controller_executable}_{slot.interceptor_id}",
            parameters=parameters,
            output='screen',
            emulate_tty=True,
        ))

    return LaunchDescription(nodes)
