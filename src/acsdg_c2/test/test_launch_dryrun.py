"""Dry-run check: c2.launch.py spawns one controller node per FLEET slot.

This catches 'launch.py forgot to iterate FLEET' regressions cheaply, with no
rclpy or ros2 launch machinery — just Python import + LaunchDescription
inspection.
"""
import importlib.util
from pathlib import Path

from launch_ros.actions import Node

from acsdg_c2.fleet import FLEET


def _import_launch_module():
    """Import the launch file as a regular Python module without running ros2 launch."""
    here = Path(__file__).resolve()
    for parent in (here, *here.parents):
        candidate = parent / "src" / "acsdg_c2" / "launch" / "c2.launch.py"
        if candidate.is_file():
            launch_path = candidate
            break
        candidate = parent / "acsdg_c2" / "launch" / "c2.launch.py"
        if candidate.is_file():
            launch_path = candidate
            break
    else:
        raise FileNotFoundError("Could not locate c2.launch.py from test working dir")
    spec = importlib.util.spec_from_file_location("c2_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[union-attr]
    return module


def test_launch_description_spawns_one_controller_per_fleet_slot():
    mod = _import_launch_module()
    desc = mod.generate_launch_description()
    nodes = [a for a in desc.entities if isinstance(a, Node)]
    controller_executables = {
        slot.controller_executable for slot in FLEET
    }
    spawned_controllers = [
        n for n in nodes if n.node_executable in controller_executables
    ]
    assert len(spawned_controllers) == len(FLEET), (
        f"Expected {len(FLEET)} controller spawns, got {len(spawned_controllers)}: "
        f"{[n.node_executable for n in spawned_controllers]}")


def _flatten_parameters(node: Node) -> dict:
    """Read a launch_ros.Node's normalized parameter dict back to {name: value}.

    launch_ros stores parameters as a tuple of dicts whose keys are
    (TextSubstitution(name),) tuples — name-mangle into _Node__parameters and
    pull each TextSubstitution.text back out. Brittle on the launch_ros
    internal layout; acceptable for a smoke test.
    """
    out: dict = {}
    raw = node._Node__parameters or ()  # type: ignore[attr-defined]
    for entry in raw:
        if not isinstance(entry, dict):
            continue
        for key, value in entry.items():
            if isinstance(key, tuple) and len(key) == 1 and hasattr(key[0], 'text'):
                out[key[0].text] = value
            elif isinstance(key, str):
                out[key] = value
    return out


def test_launch_description_passes_fleet_homes_to_controllers():
    """Every controller node must receive its FLEET slot's home as parameters."""
    mod = _import_launch_module()
    desc = mod.generate_launch_description()
    nodes = [a for a in desc.entities if isinstance(a, Node)]
    for slot in FLEET:
        matching = [
            n for n in nodes
            if n.node_executable == slot.controller_executable
            and _flatten_parameters(n).get('interceptor_id') == slot.interceptor_id
        ]
        assert len(matching) == 1, (
            f"FLEET slot {slot.weapon_id} (id={slot.interceptor_id}): expected one "
            f"matching node, got {len(matching)}")
        params = _flatten_parameters(matching[0])
        assert params.get('home_x') == slot.home[0], (
            f"slot {slot.weapon_id}: home_x mismatch {params.get('home_x')} vs {slot.home[0]}")
        assert params.get('home_y') == slot.home[1], (
            f"slot {slot.weapon_id}: home_y mismatch {params.get('home_y')} vs {slot.home[1]}")
        assert params.get('home_z') == slot.home[2], (
            f"slot {slot.weapon_id}: home_z mismatch {params.get('home_z')} vs {slot.home[2]}")
