"""WeaponSystem — the abstract base class every weapon implements.

The C2 engine treats the weapon registry uniformly through this interface;
each concrete weapon (Anvil, Coyote, DroneHunter, Skyranger) supplies its own
engagement envelope, Pkill curve, resource model, and dispatch behavior.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)


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
