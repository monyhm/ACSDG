"""Fortem DroneHunter F700 — net-capture octocopter (spec §6.3).

Multi-shot weapon with a 180 s relaunch_time cooldown after each engagement.
The cooldown is enforced via is_available() override — c2_engine reads
is_available() to decide which weapons enter the cost matrix, so a
DroneHunter that just captured a target stays out of dispatch for 3 minutes.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Tuple

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import (
    EngagementEnvelope,
    TargetClass,
    Track,
    WeaponState,
)


_DRONEHUNTER_MAX_SPEED       = 31.0      # m/s — post-2024 doubled-speed update
_DRONEHUNTER_MAX_RANGE       = 2000.0    # m — kill range
_DRONEHUNTER_MIN_RANGE       = 50.0      # m — safe-deploy distance from launcher
_DRONEHUNTER_MIN_ALT         = 0.0       # m
_DRONEHUNTER_MAX_ALT         = 4000.0    # m — class-typical for 18 kg multirotor
_DRONEHUNTER_RESOURCE_COST   = 0.20      # 0.20 first shot per spec
_DRONEHUNTER_RELAUNCH_TIME_S = 180.0     # multi-shot cooldown


_PKILL = {
    TargetClass.SMALL_QUAD:        0.85,
    TargetClass.GROUP_1_FIXED_WING: 0.70,
    TargetClass.GROUP_3_LOITERING:  0.50,
    TargetClass.SHAHED_CLASS:       0.40,
}


class DroneHunter(WeaponSystem):
    weapon_type = "net_octo"

    def __init__(self, weapon_id: str, home_position: Tuple[float, float, float]) -> None:
        self.weapon_id = weapon_id
        self._home = home_position
        self._current_position = home_position
        self._engaged_target_id: Optional[int] = None
        self._cooldown_until: float = 0.0   # epoch seconds; 0 = no cooldown active

    # ── External state hooks (called by c2_engine on InterceptorState) ──

    def update_position(self, position: Tuple[float, float, float]) -> None:
        self._current_position = position

    def mark_engaged(self, target_id: int) -> None:
        self._engaged_target_id = target_id

    def mark_idle(self) -> None:
        """Override: arrival-home transitions to cooldown, not immediate availability.

        DroneHunter's spec mandates 180 s relaunch_time after recovery — model this
        as a wall-clock cooldown that gates is_available() until elapsed.

        The was_engaged guard is critical: mark_idle() is called every IDLE tick at
        10 Hz from c2_engine. Without the guard, the cooldown would reset every tick
        and never expire.
        """
        was_engaged = self._engaged_target_id is not None
        self._engaged_target_id = None
        if was_engaged:
            self._cooldown_until = time.time() + _DRONEHUNTER_RELAUNCH_TIME_S

    def engaged_target_id(self) -> Optional[int]:
        return self._engaged_target_id

    # ── WeaponSystem interface ──────────────────────────────────────────

    def is_available(self) -> bool:
        """Override: cooldown blocks dispatch even when no target is engaged."""
        if self._engaged_target_id is not None:
            return False
        return time.time() >= self._cooldown_until

    def engagement_envelope(self) -> EngagementEnvelope:
        return EngagementEnvelope(
            min_range=_DRONEHUNTER_MIN_RANGE,
            max_range=_DRONEHUNTER_MAX_RANGE,
            min_alt=_DRONEHUNTER_MIN_ALT,
            max_alt=_DRONEHUNTER_MAX_ALT,
            max_closing_speed=_DRONEHUNTER_MAX_SPEED,
        )

    def pkill(self, target_class: TargetClass) -> float:
        return _PKILL.get(target_class, 0.0)

    def time_to_intercept(self, track: Track) -> float:
        d = track.range_from(*self._current_position)
        if d <= 0.0:
            return 0.0
        ux = (track.position[0] - self._current_position[0]) / d
        uy = (track.position[1] - self._current_position[1]) / d
        uz = (track.position[2] - self._current_position[2]) / d
        # Closing rate: -(unit_range . target_velocity).
        # Positive when target is moving toward the launcher.
        v_proj = (track.velocity[0] * (-ux)
                  + track.velocity[1] * (-uy)
                  + track.velocity[2] * (-uz))
        eff_speed = self._clamp_eff_speed(_DRONEHUNTER_MAX_SPEED, v_proj)
        return d / eff_speed

    def resource_cost(self) -> float:
        return _DRONEHUNTER_RESOURCE_COST

    def can_engage(self, track: Track) -> bool:
        if not self.is_available():
            return False
        d = track.range_from(*self._current_position)
        return self.engagement_envelope().contains(
            range_m=d, alt_m=track.position[2])

    def dispatch(self, track: Track) -> Dict[str, Any]:
        self.mark_engaged(track.track_id)
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
            ammo_remaining=None,    # multi-shot but net supply tracked externally
        )
