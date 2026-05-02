"""Tests for the assignment module — currently a thin wrapper on hungarian()."""

import math

from acsdg_c2.assignment import assign


def test_assign_3x3_minimum_cost_matching():
    cost = [
        [4.0, 1.0, 3.0],
        [2.0, 0.0, 5.0],
        [3.0, 2.0, 2.0],
    ]
    result = assign(cost)
    # Minimum cost matching is (0,1)+(1,0)+(2,2) = 1+2+2 = 5
    assert sorted(result) == [(0, 1), (1, 0), (2, 2)]


def test_assign_handles_rectangular_matrix_more_agents_than_jobs():
    cost = [
        [1.0, 5.0],
        [3.0, 2.0],
        [4.0, 4.0],
    ]
    result = assign(cost)
    # Two jobs, three agents — exactly two pairs, both within bounds
    assert len(result) == 2
    job_indices = sorted(j for _, j in result)
    assert job_indices == [0, 1]


def test_assign_empty_matrix_returns_empty():
    assert assign([]) == []


def test_assign_single_cell():
    assert assign([[1.5]]) == [(0, 0)]
