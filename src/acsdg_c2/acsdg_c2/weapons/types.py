"""Domain types shared by weapons, classifier, cost_function, dispatcher.

These are pure data structures with no ROS or Gazebo dependencies, so they can
be unit-tested without a running ROS graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple


class TargetClass(Enum):
    """The 4 threat classes from the design spec, §7."""
    SMALL_QUAD = auto()
    GROUP_1_FIXED_WING = auto()
    GROUP_3_LOITERING = auto()
    SHAHED_CLASS = auto()


@dataclass(frozen=True)
class EngagementEnvelope:
    """Per-weapon spatial envelope — within these bounds, can_engage may return True."""

    min_range: float       # m, slant range from weapon to target
    max_range: float       # m
    min_alt: float         # m, target altitude AGL
    max_alt: float         # m
    max_closing_speed: float  # m/s, weapon's own max airspeed (or muzzle velocity for guns)

    def contains(self, *, range_m: float, alt_m: float) -> bool:
        return (self.min_range <= range_m <= self.max_range
                and self.min_alt <= alt_m <= self.max_alt)


@dataclass(frozen=True)
class Track:
    """Snapshot of a fused target track at one cost-function evaluation tick."""

    track_id: int
    position: Tuple[float, float, float]   # (x, y, z) in world frame
    velocity: Tuple[float, float, float]   # (vx, vy, vz) in world frame
    threat_score: float                    # [0, 1] from threat_priority module
    state: str                             # FusedTarget.state ("DETECTED", "TARGETED", ...)

    def range_from(self, x: float, y: float, z: float) -> float:
        dx = self.position[0] - x
        dy = self.position[1] - y
        dz = self.position[2] - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def with_score(self, score: float) -> "Track":
        """Return a copy of this Track with `threat_score` replaced.

        Frozen dataclass forces a copy; this is the canonical way to
        rebuild a Track when downstream computation produces a new score.
        """
        from dataclasses import replace
        return replace(self, threat_score=score)


@dataclass
class WeaponState:
    """Snapshot of a weapon's runtime state — used for visualization and AI decisions."""

    weapon_id: str
    weapon_type: str           # "kinetic_quad" / "frag_jet" / "net_octo" / "gun_turret"
    position: Tuple[float, float, float]
    available: bool
    ammo_remaining: Optional[int]   # None for one-shot weapons, count for multi-shot
