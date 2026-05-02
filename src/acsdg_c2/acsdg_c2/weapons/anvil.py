"""Anduril Anvil — kinetic-quadcopter kamikaze interceptor (DoD Group 1/2).

Spec values from §6.1 of the design doc:
  max_speed = 45 m/s, max_range = 1.5 km (LOS-tethered),
  resource_cost = 0.05, one-shot, collision kill at <8 m.
Pkill table:
  small-quad 0.95, group-1-fixed-wing 0.85, group-3-loitering 0.70, shahed 0.40.
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


_ANVIL_PKILL = {
    TargetClass.SMALL_QUAD: 0.95,
    TargetClass.GROUP_1_FIXED_WING: 0.85,
    TargetClass.GROUP_3_LOITERING: 0.70,
    TargetClass.SHAHED_CLASS: 0.40,
}

_ANVIL_MAX_SPEED = 45.0       # m/s — spec §6.1
_ANVIL_MAX_RANGE = 1500.0     # m — Phase-1 sim value, spec §6.1
_ANVIL_MIN_RANGE = 5.0        # m — collision-kill close-in
_ANVIL_RESOURCE_COST = 0.05


class Anvil(WeaponSystem):

    weapon_type = "kinetic_quad"

    def __init__(
        self,
        weapon_id: str,
        home_position: Tuple[float, float, float],
    ) -> None:
        self.weapon_id = weapon_id
        self._home = home_position
        self._current_position = home_position
        self._engaged_target_id: Optional[int] = None

    # ── External state hooks (called by c2_engine on InterceptorState updates) ─

    def update_position(self, position: Tuple[float, float, float]) -> None:
        self._current_position = position

    def mark_engaged(self, target_id: int) -> None:
        self._engaged_target_id = target_id

    def mark_idle(self) -> None:
        self._engaged_target_id = None

    def engaged_target_id(self) -> Optional[int]:
        """The track id this weapon is currently engaging, or None if available."""
        return self._engaged_target_id

    # ── WeaponSystem interface ────────────────────────────────────────────

    def is_available(self) -> bool:
        return self._engaged_target_id is None

    def engagement_envelope(self) -> EngagementEnvelope:
        return EngagementEnvelope(
            min_range=_ANVIL_MIN_RANGE,
            max_range=_ANVIL_MAX_RANGE,
            min_alt=0.0,
            max_alt=1066.0,        # DoD Group 2 ceiling (spec §6.1)
            max_closing_speed=_ANVIL_MAX_SPEED,
        )

    def pkill(self, target_class: TargetClass) -> float:
        return _ANVIL_PKILL[target_class]

    def time_to_intercept(self, track: Track) -> float:
        d = track.range_from(*self._current_position)
        if d <= 0.0:
            return 0.0
        # closing rate along line of sight (positive = inbound)
        ux = (track.position[0] - self._current_position[0]) / d
        uy = (track.position[1] - self._current_position[1]) / d
        uz = (track.position[2] - self._current_position[2]) / d
        # target velocity projected toward the weapon (inbound = negative)
        v_proj = (track.velocity[0] * (-ux)
                  + track.velocity[1] * (-uy)
                  + track.velocity[2] * (-uz))
        eff_speed = max(1.0, _ANVIL_MAX_SPEED + v_proj)
        return d / eff_speed

    def resource_cost(self) -> float:
        return _ANVIL_RESOURCE_COST

    def can_engage(self, track: Track) -> bool:
        if not self.is_available():
            return False
        d = track.range_from(*self._current_position)
        return self.engagement_envelope().contains(
            range_m=d, alt_m=track.position[2])

    def dispatch(self, track: Track) -> Dict[str, Any]:
        self.mark_engaged(track.track_id)
        # Truncating int(...) matches the legacy c2_engine_node priority byte
        # exactly (see legacy line ~195) — round() would diverge by one ULP at
        # the half-byte boundary, breaking byte-for-byte regression with the
        # pre-refactor demo.
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
            ammo_remaining=None,    # Anvil is one-shot
        )
