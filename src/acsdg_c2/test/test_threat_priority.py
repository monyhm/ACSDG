"""Tests for threat priority scoring — must match the legacy _score()."""

import math
import pytest

from acsdg_c2.cost_function.threat_priority import score_threat
from acsdg_c2.weapons.types import Track


def make_track(pos, vel, threat_score=0.0, state="DETECTED", track_id=1):
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=threat_score, state=state)


def test_score_at_origin_with_no_velocity_is_proximity_only():
    """A target at the origin gets full proximity score and no heading/speed."""
    t = make_track(pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    # proximity = 1.0, heading = 0, speed = 0 → 0.5*1 + 0.3*0 + 0.2*0 = 0.5
    assert score_threat(t) == pytest.approx(0.5)


def test_score_far_target_zero_proximity():
    """Beyond MAX_RANGE the proximity term saturates at 0."""
    t = make_track(pos=(500.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    # MAX_RANGE = 400, so proximity = 0; heading = 0; speed = 0 → 0
    assert score_threat(t) == pytest.approx(0.0)


def test_score_inbound_target_credits_heading():
    """Target heading toward the origin gets full heading bonus."""
    # at (200, 0, 0) moving at (-10, 0, 0): heading vector exactly aligned
    t = make_track(pos=(200.0, 0.0, 0.0), vel=(-10.0, 0.0, 0.0))
    # proximity = 1 - 200/400 = 0.5
    # heading = 1.0 (perfect inbound)
    # speed_s = min(1, 10/20) = 0.5
    # → 0.5*0.5 + 0.3*1.0 + 0.2*0.5 = 0.25 + 0.30 + 0.10 = 0.65
    assert score_threat(t) == pytest.approx(0.65, rel=1e-3)


def test_score_outbound_target_zero_heading():
    """Target moving away contributes no heading score."""
    t = make_track(pos=(200.0, 0.0, 0.0), vel=(10.0, 0.0, 0.0))
    # proximity = 0.5, heading = 0 (max(0, ...) clips), speed_s = 0.5
    # → 0.25 + 0 + 0.10 = 0.35
    assert score_threat(t) == pytest.approx(0.35, rel=1e-3)


def test_score_speed_saturates_at_max_speed():
    """Speed term clamps at 1.0 once track speed exceeds MAX_SPEED."""
    t = make_track(pos=(200.0, 0.0, 0.0), vel=(-100.0, 0.0, 0.0))
    # proximity = 0.5, heading = 1, speed_s = 1
    # → 0.25 + 0.30 + 0.20 = 0.75
    assert score_threat(t) == pytest.approx(0.75, rel=1e-3)
