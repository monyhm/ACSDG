"""Dispatcher — converts a (weapon, track) decision into an EngagementOrder payload.

Phase 1: every weapon is an Anvil, identified `anvil_0`..`anvil_3`. The legacy
interceptor controller subscribes to /interceptors/unit_{1..4}/state and expects
EngagementOrder.interceptor_id ∈ {1, 2, 3, 4}. We map "anvil_<N>" → N+1 so the
existing C++ controller keeps working without modification.

In Phase 4, when heterogeneous weapons appear, this mapping becomes
weapon-id-aware (each weapon controller gets its own namespace).
"""

from __future__ import annotations

from typing import Any, Dict

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import Track


class Dispatcher:

    def translate(self, weapon: WeaponSystem, track: Track) -> Dict[str, Any]:
        """Issue weapon.dispatch() and return the payload for EngagementOrder."""
        payload = weapon.dispatch(track)
        # Phase 1: extract numeric tail of "anvil_N" and add 1 → unit_{1..4}
        wid = payload.get("weapon_id", "")
        try:
            numeric_tail = int(wid.split("_")[-1])
            interceptor_id = numeric_tail + 1
        except (ValueError, IndexError):
            interceptor_id = 0   # caller treats 0 as "unknown / drop"
        priority = int(min(255, max(0, payload.get("priority", 0))))
        return {
            "target_id": int(payload["target_id"]),
            "interceptor_id": interceptor_id,
            "priority": priority,
        }
