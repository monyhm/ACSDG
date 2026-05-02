"""Raytheon Coyote Block 2 — frag-warhead jet interceptor.

Spec values from §6.2 of the design doc:
  max_speed = 160 m/s (Mach 0.45), max_range = 5 km in sim,
  resource_cost = 0.10 ($100k FY24 unit), one-shot,
  proximity-fuze frag warhead with Pk≥0.5 at 5 m, Pk≥0.3 at 8 m.
Pkill table:
  small-quad 0.60, group-1-fixed-wing 0.85, group-3-loitering 0.95, shahed 0.95.

Phase-2 contract:
- Lives in the same weapon registry as Anvil (composes through WeaponSystem ABC).
- weapon_id "coyote_<N>" maps to interceptor_id (N+1) via Dispatcher's existing
  numeric-tail logic. Coyote takes the Anvil-vacated NE slot (slot 1), so
  weapon_id "coyote_0" → interceptor_id 1.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)


_COYOTE_PKILL = {
    TargetClass.SMALL_QUAD: 0.60,
    TargetClass.GROUP_1_FIXED_WING: 0.85,
    TargetClass.GROUP_3_LOITERING: 0.95,
    TargetClass.SHAHED_CLASS: 0.95,
}

_COYOTE_MAX_SPEED = 160.0      # m/s — spec §6.2 (Mach 0.45)
_COYOTE_MAX_RANGE = 5000.0     # m — Phase-2 sim value (real 10–15 km), spec §6.2
_COYOTE_MIN_RANGE = 100.0      # m — proximity fuze arming distance, spec §6.2
_COYOTE_RESOURCE_COST = 0.10   # $100k normalized — spec §6.2


class Coyote(WeaponSystem):

    weapon_type = "frag_jet"

    def __init__(
        self,
        weapon_id: str,
        home_position: Tuple[float, float, float],
    ) -> None:
        self.weapon_id = weapon_id
        self._home = home_position
        self._current_position = home_position
        self._engaged_target_id: Optional[int] = None

    # ── External state hooks (called by c2_engine on InterceptorState) ──

    def update_position(self, position: Tuple[float, float, float]) -> None:
        self._current_position = position

    def mark_engaged(self, target_id: int) -> None:
        self._engaged_target_id = target_id

    def mark_idle(self) -> None:
        self._engaged_target_id = None

    def engaged_target_id(self) -> Optional[int]:
        return self._engaged_target_id

    # ── WeaponSystem interface ──────────────────────────────────────────

    def is_available(self) -> bool:
        return self._engaged_target_id is None

    def engagement_envelope(self) -> EngagementEnvelope:
        return EngagementEnvelope(
            min_range=_COYOTE_MIN_RANGE,
            max_range=_COYOTE_MAX_RANGE,
            min_alt=0.0,
            max_alt=4500.0,        # spec §6.2 sim altitude ceiling
            max_closing_speed=_COYOTE_MAX_SPEED,
        )

    def pkill(self, target_class: TargetClass) -> float:
        return _COYOTE_PKILL[target_class]

    def time_to_intercept(self, track: Track) -> float:
        d = track.range_from(*self._current_position)
        if d <= 0.0:
            return 0.0
        ux = (track.position[0] - self._current_position[0]) / d
        uy = (track.position[1] - self._current_position[1]) / d
        uz = (track.position[2] - self._current_position[2]) / d
        v_proj = (track.velocity[0] * (-ux)
                  + track.velocity[1] * (-uy)
                  + track.velocity[2] * (-uz))
        eff_speed = max(1.0, _COYOTE_MAX_SPEED + v_proj)
        return d / eff_speed

    def resource_cost(self) -> float:
        return _COYOTE_RESOURCE_COST

    def can_engage(self, track: Track) -> bool:
        if not self.is_available():
            return False
        d = track.range_from(*self._current_position)
        return self.engagement_envelope().contains(
            range_m=d, alt_m=track.position[2])

    def dispatch(self, track: Track) -> Dict[str, Any]:
        self.mark_engaged(track.track_id)
        # Truncating int(...) matches Anvil's priority byte exactly so a
        # heterogeneous wave produces consistent EngagementOrder.priority bytes.
        return {
            "target_id": track.track_id,
            "weapon_id": self.weapon_id,
            "priority": int(min(255, max(0, int(track.threat_score * 255)))),
        }

    def state(self) -> WeaponState:
        return WeaponState(
            weapon_id=self.weapon_id,
            weapon_type=self.weapon_type,
            position=self._current_position,
            available=self.is_available(),
            ammo_remaining=None,    # Coyote is one-shot, like Anvil
        )
