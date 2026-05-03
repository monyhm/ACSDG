"""Dispatcher — converts a (weapon, track) decision into an EngagementOrder payload.

Phase 1: every weapon was an Anvil, identified `anvil_0`..`anvil_3`. A numeric-tail
heuristic mapped "anvil_<N>" → N+1. Phase 3 adds DroneHunter ("dronehunter_0") on
interceptor_id=3 — the numeric-tail logic would have wrongly returned 1 (colliding
with Coyote). The mapping is now FLEET-driven: weapon_id → interceptor_id is looked
up from fleet.FLEET, the single source of truth for fleet composition.
"""

from __future__ import annotations

from typing import Any, Dict

from acsdg_c2.fleet import FLEET
from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import Track


class Dispatcher:

    def translate(self, weapon: WeaponSystem, track: Track) -> Dict[str, Any]:
        """Issue weapon.dispatch() and return the payload for EngagementOrder.

        interceptor_id is sourced from FLEET — single source of truth for the
        weapon_id → slot mapping. Replaces the Phase 1 numeric-tail heuristic
        that broke when Phase 3 added a non-Anvil weapon at slot 3.
        """
        payload = weapon.dispatch(track)
        wid = payload.get("weapon_id", "")
        try:
            interceptor_id = next(
                s.interceptor_id for s in FLEET if s.weapon_id == wid)
        except StopIteration:
            raise ValueError(
                f"Dispatcher: weapon_id={wid!r} not found in FLEET. "
                f"Add it to acsdg_c2/fleet.py FLEET tuple.")
        priority = int(min(255, max(0, payload.get("priority", 0))))
        return {
            "target_id": int(payload["target_id"]),
            "interceptor_id": interceptor_id,
            "priority": priority,
        }
