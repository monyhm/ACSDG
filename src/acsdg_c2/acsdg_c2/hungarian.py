"""
hungarian.py — Pure-Python Munkres / Hungarian algorithm.

Solves the linear assignment problem:
    minimise  Σ cost[agent][assignment[agent]]
subject to one-to-one matching.

No external dependencies.  O(n³) time, O(n²) space.
"""

from typing import List, Tuple


def hungarian(cost_matrix: List[List[float]]) -> List[Tuple[int, int]]:
    """Find the minimum-cost assignment for a rectangular cost matrix.

    Parameters
    ----------
    cost_matrix:
        m × n matrix; entry [i][j] = cost of assigning agent i to job j.
        Accepts any numeric values (int or float).

    Returns
    -------
    List of (agent_index, job_index) pairs giving the optimal assignment.
    Length is min(m, n).  Indices refer to the *original* matrix dimensions.

    Example
    -------
    >>> hungarian([[4, 1, 3], [2, 0, 5], [3, 2, 2]])
    [(0, 1), (1, 0), (2, 2)]   # total cost = 1 + 2 + 2 = 5
    """
    if not cost_matrix or not cost_matrix[0]:
        return []

    orig_m: int = len(cost_matrix)
    orig_n: int = len(cost_matrix[0])
    n: int      = max(orig_m, orig_n)
    EPS: float  = 1e-9  # floating-point zero tolerance

    # ── Build padded square matrix ────────────────────────────────────────
    # Padding cells cost 0 so they don't attract real assignments.
    C: List[List[float]] = [[0.0] * n for _ in range(n)]
    for i in range(orig_m):
        for j in range(orig_n):
            C[i][j] = float(cost_matrix[i][j])

    # ── Step 1: subtract row minima ───────────────────────────────────────
    for i in range(n):
        row_min = min(C[i])
        C[i] = [v - row_min for v in C[i]]

    # ── Step 2: subtract column minima ────────────────────────────────────
    for j in range(n):
        col_min = min(C[i][j] for i in range(n))
        for i in range(n):
            C[i][j] -= col_min

    # ── Marking arrays (0 = none, 1 = starred, 2 = primed) ───────────────
    mark    = [[0] * n for _ in range(n)]
    row_cov = [False] * n
    col_cov = [False] * n

    # ── Step 3: greedily star one zero per row and column ─────────────────
    for i in range(n):
        for j in range(n):
            if C[i][j] < EPS and not row_cov[i] and not col_cov[j]:
                mark[i][j] = 1
                row_cov[i] = True
                col_cov[j] = True
    row_cov[:] = [False] * n
    col_cov[:] = [False] * n

    # ── Helper closures ───────────────────────────────────────────────────

    def _cover_starred_cols() -> None:
        for j in range(n):
            col_cov[j] = any(mark[i][j] == 1 for i in range(n))

    def _find_uncovered_zero() -> Tuple[int, int]:
        for i in range(n):
            if row_cov[i]:
                continue
            for j in range(n):
                if C[i][j] < EPS and not col_cov[j]:
                    return i, j
        return -1, -1

    def _starred_in_row(r: int) -> int:
        for j in range(n):
            if mark[r][j] == 1:
                return j
        return -1

    def _starred_in_col(c: int) -> int:
        for i in range(n):
            if mark[i][c] == 1:
                return i
        return -1

    def _augment_path(r0: int, c0: int) -> None:
        """Follow primed→starred alternating path from (r0, c0), flip it."""
        path = [(r0, c0)]
        while True:
            sr = _starred_in_col(path[-1][1])
            if sr < 0:
                break
            path.append((sr, path[-1][1]))
            # find the prime in that row
            pc = next(j for j in range(n) if mark[sr][j] == 2)
            path.append((sr, pc))
        for r, c in path:
            mark[r][c] = 0 if mark[r][c] == 1 else 1  # flip star / un-star

    # ── Main Munkres loop ─────────────────────────────────────────────────
    _cover_starred_cols()

    while not all(col_cov):
        ri, ci = _find_uncovered_zero()

        if ri < 0:
            # No uncovered zero: adjust cost matrix
            uncov_vals = [
                C[i][j]
                for i in range(n) for j in range(n)
                if not row_cov[i] and not col_cov[j]
            ]
            if not uncov_vals:
                break  # degenerate matrix — shouldn't happen on valid input
            min_val = min(uncov_vals)
            for i in range(n):
                for j in range(n):
                    if not row_cov[i] and not col_cov[j]:
                        C[i][j] -= min_val      # uncovered  → subtract
                    if row_cov[i] and col_cov[j]:
                        C[i][j] += min_val      # double-covered → add
            continue

        # Prime the uncovered zero
        mark[ri][ci] = 2
        sc = _starred_in_row(ri)

        if sc >= 0:
            # There is a star in this row: cover row, uncover star's column
            row_cov[ri] = True
            col_cov[sc] = False
        else:
            # No star in row → augment and reset
            _augment_path(ri, ci)
            for i in range(n):
                for j in range(n):
                    if mark[i][j] == 2:
                        mark[i][j] = 0
            row_cov[:] = [False] * n
            col_cov[:] = [False] * n
            _cover_starred_cols()

    # ── Extract assignments (original matrix bounds only) ─────────────────
    result: List[Tuple[int, int]] = []
    for i in range(orig_m):
        for j in range(orig_n):
            if mark[i][j] == 1:
                result.append((i, j))
    return result
