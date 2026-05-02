"""Assignment solver wrapper.

Phase 1 delegates to the existing pure-Python Hungarian implementation in
acsdg_c2.hungarian. Later phases may swap in scipy.optimize.linear_sum_assignment
or a custom solver without callers having to know.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from acsdg_c2.hungarian import hungarian


def assign(cost_matrix: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    """Return the min-cost (agent, job) assignment.

    Length is min(rows, cols). Indices refer to the original matrix dimensions.
    Empty input returns an empty list.
    """
    return hungarian([list(row) for row in cost_matrix])
