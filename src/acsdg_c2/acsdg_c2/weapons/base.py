"""WeaponSystem — the abstract base class every weapon implements.

The C2 engine treats the weapon registry uniformly through this interface;
each concrete weapon (Anvil, Coyote, DroneHunter, Skyranger) supplies its own
engagement envelope, Pkill curve, resource model, and dispatch behavior.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)

# Module-level constant: stern-aspect-target ToI floor as fraction of max_speed
_EFF_SPEED_FLOOR_FRAC = 0.5


class WeaponSystem(ABC):

    weapon_id: str       # unique e.g. "anvil_0", "coyote_0"
    weapon_type: str     # "kinetic_quad" / "frag_jet" / "net_octo" / "gun_turret"

    # ── Capability queries ───────────────────────────────────────────────

    @abstractmethod
    def is_available(self) -> bool:
        """True if this weapon has resources and is not currently engaged."""

    @abstractmethod
    def engagement_envelope(self) -> EngagementEnvelope:
        """Spatial bounds within which can_engage may return True."""

    @abstractmethod
    def pkill(self, target_class: TargetClass) -> float:
        """Per-class kill probability (0.0–1.0). Lookup from spec table."""

    @abstractmethod
    def time_to_intercept(self, track: Track) -> float:
        """Geometric ETA in seconds. Drone: lead-pursuit time. Gun: ballistic ToF."""

    @abstractmethod
    def resource_cost(self) -> float:
        """Normalized $cost or unit-deplete penalty per dispatch."""

    @abstractmethod
    def can_engage(self, track: Track) -> bool:
        """In envelope AND available AND has ammo (for guns)."""

    # ── Dispatch and state ────────────────────────────────────────────────

    @abstractmethod
    def dispatch(self, track: Track) -> Dict[str, Any]:
        """Issue the engagement command. Decrements resources / sets engaged flag.

        Returns a dict with keys {target_id, weapon_id, priority, ...} that the
        Dispatcher converts into an EngagementOrder ROS message.
        """

    @abstractmethod
    def state(self) -> WeaponState:
        """Snapshot for visualization and AI decisions."""

    @abstractmethod
    def engaged_target_id(self) -> Optional[int]:
        """Track id this weapon is currently engaging, or None if available.

        Used by the C2 engine's dispatch loop to dedupe — a track already
        being engaged by some weapon is excluded from re-assignment. For
        one-shot kinetic weapons (Anvil, Coyote) this is a single id; for
        multi-shot weapons (Skyranger, DroneHunter post-recovery) Phase 4
        will need richer semantics — see Phase 2 plan TODOs.
        """

    # ── Launch-time hooks ────────────────────────────────────────────────

    def launch_parameters(self) -> list[dict]:
        """Per-instance ROS parameters this weapon's controller needs at launch.

        Default: no extra parameters beyond the standard interceptor_id +
        home_x/y/z that every controller receives. Override in subclasses
        (e.g., Coyote needs pkill_small_quad + rng_seed for the frag-fuze).

        Concrete method, not abstract — subclasses inherit `[]` unless they
        have per-class launch needs. This keeps c2.launch.py free of
        per-class `if/elif` ladders as new weapons are added.
        """
        return []

    def _clamp_eff_speed(self, max_speed: float, v_proj: float) -> float:
        """Effective closing speed for time_to_intercept, clamped at half max_speed.

        For closing engagements (v_proj > 0), returns max_speed + v_proj.
        For stern-aspect / fleeing targets where max_speed + v_proj < 0.5*max_speed,
        returns 0.5*max_speed — yielding a finite, seconds-unit ToI rather than the
        eff_speed=1.0 sentinel that produced garbage units in the cost matrix.
        """
        return max(_EFF_SPEED_FLOOR_FRAC * max_speed, max_speed + v_proj)
