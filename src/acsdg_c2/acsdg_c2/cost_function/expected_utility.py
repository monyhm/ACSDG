"""Cost-matrix builder for C2 weapon-target pairing.

Phase 1 entry: cost = time_to_intercept(weapon, track), with +inf for
out-of-envelope or unavailable pairs. Phase 5 will replace this body with
the full expected-utility math (Pkill × class posterior, resource cost,
time-to-intercept, threat priority) per spec §11.
"""

from __future__ import annotations

import math
from typing import List, Sequence

from acsdg_c2.weapons.base import WeaponSystem
from acsdg_c2.weapons.types import Track


def build_cost_matrix(
    weapons: Sequence[WeaponSystem],
    tracks: Sequence[Track],
) -> List[List[float]]:
    """Return an m×n cost matrix where m=len(weapons), n=len(tracks).

    Cost is approximate time-to-intercept in seconds; +inf flags forbidden
    assignments (weapon unavailable or target outside engagement envelope).
    """
    matrix: List[List[float]] = []
    for w in weapons:
        row: List[float] = []
        for t in tracks:
            if not w.can_engage(t):
                row.append(math.inf)
            else:
                row.append(w.time_to_intercept(t))
        matrix.append(row)
    return matrix
