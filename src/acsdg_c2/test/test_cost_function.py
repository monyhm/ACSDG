"""Tests for the Phase-1 cost-matrix builder (time-to-intercept geometry)."""

import math
import pytest

from acsdg_c2.cost_function.expected_utility import build_cost_matrix
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.types import Track


def make_track(track_id, pos, vel=(0.0, 0.0, 0.0), threat=0.5):
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=threat, state="DETECTED")


def test_cost_matrix_shape_matches_inputs():
    weapons = [Anvil("anvil_0", (0.0, 0.0, 20.0)),
               Anvil("anvil_1", (1000.0, 0.0, 20.0))]
    tracks = [make_track(1, (500.0, 0.0, 50.0)),
              make_track(2, (1500.0, 0.0, 50.0)),
              make_track(3, (300.0, 0.0, 50.0))]
    matrix = build_cost_matrix(weapons, tracks)
    assert len(matrix) == 2
    assert all(len(row) == 3 for row in matrix)


def test_cost_matrix_uses_time_to_intercept():
    a = Anvil("anvil_0", (0.0, 0.0, 0.0))
    track = make_track(1, (450.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    matrix = build_cost_matrix([a], [track])
    # eff_speed = 45 m/s (max_speed) + 0 closing = 45; ToI = 450/45 = 10 s
    assert matrix[0][0] == pytest.approx(10.0, rel=0.01)


def test_cost_matrix_marks_unavailable_weapon_as_infinite():
    a = Anvil("anvil_0", (0.0, 0.0, 0.0))
    a.mark_engaged(target_id=99)   # weapon already busy
    track = make_track(1, (500.0, 0.0, 0.0))
    matrix = build_cost_matrix([a], [track])
    assert math.isinf(matrix[0][0])


def test_cost_matrix_marks_out_of_envelope_as_infinite():
    a = Anvil("anvil_0", (0.0, 0.0, 0.0))
    track = make_track(1, (5000.0, 0.0, 0.0))   # > 1.5 km Anvil max range
    matrix = build_cost_matrix([a], [track])
    assert math.isinf(matrix[0][0])


def test_empty_inputs_return_empty_matrix():
    assert build_cost_matrix([], []) == []
    assert build_cost_matrix([Anvil("a", (0, 0, 0))], []) == [[]]
    assert build_cost_matrix([], [make_track(1, (0, 0, 0))]) == []
