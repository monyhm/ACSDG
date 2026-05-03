"""ADAMANT v2 6-component shaped reward.

Components: survival, coverage, depletion, kill_bonus, waste_penalty, geometry.
The geometry component requires kill events to carry "geometry_score"
(1.0 - |cos(angle)| between interceptor and threat velocity vectors at the
moment of intercept). The v2 environment populates this field. If a kill event
lacks the field, a warning is logged once per process and the geometry
component contributes 0.0 for that kill.
"""

from __future__ import annotations

import math
import warnings
from typing import Any


_ASSET_X = 50.0
_ASSET_Y = 50.0
_GRID_UNIT_M = 200.0  # 1 grid unit = 200 m

_geometry_warning_logged = False


def compute_reward_v2(
    events: dict[str, Any],
    state: dict[str, Any],
    terminated: bool,
    truncated: bool,
) -> dict[str, float]:
    survival = _survival(state, terminated)
    coverage = _coverage(state)
    depletion = _depletion(state)
    kill_bonus = _kill_bonus(events)
    waste_penalty = _waste_penalty(events)
    geometry = _geometry(events)

    total = survival + coverage + depletion + kill_bonus + waste_penalty + geometry
    components = {
        "total": total,
        "survival": survival,
        "coverage": coverage,
        "depletion": depletion,
        "kill_bonus": kill_bonus,
        "waste_penalty": waste_penalty,
        "geometry": geometry,
    }
    others = sum(v for k, v in components.items() if k != "total")
    assert abs(total - others) < 1e-6, f"reward total {total} != sum {others}"
    return components


def _survival(state: dict, terminated: bool) -> float:
    # Truncation alone (step limit, threats remaining) gets 0 — we don't reward
    # running out the clock.
    if not terminated:
        return 0.0
    return -200.0 if state.get("asset_hit", False) else 200.0


def _coverage(state: dict) -> float:
    # Quadrant bounds use >= on one side per quadrant so that axis-sitting
    # interceptor bases (INT-NORTH at x=50, INT-EAST at y=50, etc.) count toward
    # sector coverage of an adjacent quadrant. The default bases are designed as
    # omnidirectional defense points; under strict ">" they would never count
    # toward any quadrant and coverage would be structurally 0.
    #   NE: x >= 50 AND y >= 50
    #   NW: x <  50 AND y >= 50
    #   SW: x <  50 AND y <  50
    #   SE: x >= 50 AND y <  50
    quadrants = set()
    for intc in state.get("interceptors", []):
        if intc.get("status") not in ("READY", "INTERCEPT"):
            continue
        x, y = intc["x"], intc["y"]
        if x >= _ASSET_X and y >= _ASSET_Y:
            quadrants.add("NE")
        elif x < _ASSET_X and y >= _ASSET_Y:
            quadrants.add("NW")
        elif x < _ASSET_X and y < _ASSET_Y:
            quadrants.add("SW")
        elif x >= _ASSET_X and y < _ASSET_Y:
            quadrants.add("SE")
    return 0.2 * len(quadrants)


def _depletion(state: dict) -> float:
    available = sum(
        1 for i in state.get("interceptors", [])
        if i.get("status") == "READY"
    )
    if available == 0:
        return -1.0
    if available == 1:
        return -0.3
    return 0.0


def _kill_bonus(events: dict) -> float:
    bonus = 0.0
    for kill in events.get("kills", []):
        kx = kill.get("kill_x", _ASSET_X)
        ky = kill.get("kill_y", _ASSET_Y)
        dist_grid = math.hypot(kx - _ASSET_X, ky - _ASSET_Y)
        dist_km = dist_grid * _GRID_UNIT_M / 1000.0
        bonus += max(0.0, (dist_km - 3.0) * 5.0)
    return bonus


def _waste_penalty(events: dict) -> float:
    # Multi-engagement waste only. Single-engagement misses are honest attempts
    # and are not penalized here.
    return -10.0 * len(events.get("wasted_interceptors", []))


def _geometry(events: dict) -> float:
    global _geometry_warning_logged
    score = 0.0
    for kill in events.get("kills", []):
        if "geometry_score" not in kill:
            if not _geometry_warning_logged:
                warnings.warn(
                    "compute_reward_v2: kill event missing 'geometry_score'; "
                    "geometry component returns 0.0 for that kill. Update the "
                    "environment to attach intercept geometry to kill events.",
                    stacklevel=2,
                )
                _geometry_warning_logged = True
            continue
        gs = kill["geometry_score"]
        if gs > 0.7:
            score += 1.0
        elif gs < 0.4:
            score -= 1.0
    return score
