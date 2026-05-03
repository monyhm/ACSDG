"""ACSDG fleet — single source of truth for which interceptor occupies which slot.

Consumers:
  - c2_engine_node.py        : iterates FLEET to instantiate self._weapons
  - gz_bridge_shim.py        : consumes bridged_models() for ROS↔gz wiring
  - c2.launch.py             : iterates FLEET to spawn the right controller per slot
  - military_base.sdf        : agreement enforced via assert_matches_sdf() at startup

Adding a weapon: append one Slot. Validator + SDF-agreement test gate the add.
"""

from dataclasses import dataclass
from typing import Tuple, Type

from acsdg_c2.weapons import Anvil, Coyote, WeaponSystem


@dataclass(frozen=True)
class Slot:
    """One occupied launcher post in the heterogeneous fleet."""
    interceptor_id: int                  # 1..N — feeds /interceptors/unit_<id>/* topics
    weapon_class: Type[WeaponSystem]
    weapon_id: str                       # "anvil_2", "coyote_0" — dispatcher numeric-tail key
    controller_executable: str           # ament-installed binary name
    gz_model_kind: str                   # SDF model name root: "interceptor" | "coyote"
    gz_instance_index: int               # 1..M within that kind, used in /model/<kind>_<i>/*
    home: Tuple[float, float, float]     # spawn pose, MUST agree with the SDF


def _validate_fleet(fleet: Tuple[Slot, ...]) -> None:
    """Reject misconfigured fleets at import time. Catches typos that would
    otherwise silently make two controllers fight over the same Gazebo body
    or the same /interceptors/unit_<id>/* topic stream."""
    seen_ids: set[int] = set()
    seen_weapon_ids: set[str] = set()
    seen_gz: set[Tuple[str, int]] = set()
    seen_homes: set[Tuple[float, float, float]] = set()
    for s in fleet:
        if s.interceptor_id in seen_ids:
            raise ValueError(f"FLEET: duplicate interceptor_id={s.interceptor_id}")
        if s.weapon_id in seen_weapon_ids:
            raise ValueError(f"FLEET: duplicate weapon_id={s.weapon_id!r}")
        gz_key = (s.gz_model_kind, s.gz_instance_index)
        if gz_key in seen_gz:
            raise ValueError(
                f"FLEET: duplicate gz model {s.gz_model_kind}_{s.gz_instance_index}")
        if s.home in seen_homes:
            raise ValueError(f"FLEET: duplicate home pose {s.home}")
        seen_ids.add(s.interceptor_id)
        seen_weapon_ids.add(s.weapon_id)
        seen_gz.add(gz_key)
        seen_homes.add(s.home)


FLEET: Tuple[Slot, ...] = (
    Slot(1, Coyote, "coyote_0", "coyote_controller_node",      "coyote",      1, ( 177.0,  177.0, 20.0)),
    Slot(2, Anvil,  "anvil_1",  "interceptor_controller_node", "interceptor", 2, (-177.0,  177.0, 20.0)),
    Slot(3, Anvil,  "anvil_2",  "interceptor_controller_node", "interceptor", 3, ( 177.0, -177.0, 20.0)),
    Slot(4, Anvil,  "anvil_3",  "interceptor_controller_node", "interceptor", 4, (-177.0, -177.0, 20.0)),
)


_validate_fleet(FLEET)   # runs at import; fails fast on misconfiguration


import xml.etree.ElementTree as ET
from typing import Dict


def parse_sdf_includes(sdf_path: str) -> Dict[str, Tuple[float, float, float]]:
    """Walk an SDF world file and return {model_name: (x, y, z)} for every <include>.

    Pose strings are six floats 'x y z roll pitch yaw'; we keep only x, y, z.
    Returns an empty dict if no <include> blocks are present.
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    out: Dict[str, Tuple[float, float, float]] = {}
    # SDF doesn't use namespaces in our worlds; iterate any descendant <include>.
    for inc in root.iter("include"):
        name_el = inc.find("name")
        pose_el = inc.find("pose")
        if name_el is None or pose_el is None:
            continue
        name = (name_el.text or "").strip()
        if not name:
            continue
        pose_text = (pose_el.text or "").strip()
        parts = pose_text.split()
        if len(parts) < 3:
            continue
        try:
            xyz = (float(parts[0]), float(parts[1]), float(parts[2]))
        except ValueError:
            continue
        out[name] = xyz
    return out


def assert_matches_sdf(sdf_path: str, abs_tol: float = 0.01) -> None:
    """Raise AssertionError if any FLEET.home disagrees with the SDF spawn pose
    for the same model name.

    Called at c2_engine_node startup (defense in depth) and in the regression
    test (CI catches divergence before launch). Tolerance is 0.01 m absolute.
    """
    poses = parse_sdf_includes(sdf_path)
    for slot in FLEET:
        name = f"{slot.gz_model_kind}_{slot.gz_instance_index}"
        assert name in poses, (
            f"FLEET slot {slot.weapon_id!r} expects SDF spawn for {name!r}, "
            f"but no <include> with that name was found in {sdf_path}")
        sx, sy, sz = poses[name]
        hx, hy, hz = slot.home
        for axis, sv, hv in (("x", sx, hx), ("y", sy, hy), ("z", sz, hz)):
            assert abs(sv - hv) <= abs_tol, (
                f"{name}: SDF.{axis}={sv} disagrees with FLEET.home.{axis}={hv} "
                f"(slot weapon_id={slot.weapon_id!r}, tol={abs_tol}m)")
