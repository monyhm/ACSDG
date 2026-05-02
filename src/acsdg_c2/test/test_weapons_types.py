"""Tests for domain types used by the weapons / cost-function modules."""

import math
import pytest

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)


def test_target_class_enum_has_four_phase_taxonomy_classes():
    assert {c.name for c in TargetClass} == {
        "SMALL_QUAD",
        "GROUP_1_FIXED_WING",
        "GROUP_3_LOITERING",
        "SHAHED_CLASS",
    }


def test_engagement_envelope_in_range_within_bounds():
    env = EngagementEnvelope(
        min_range=100.0, max_range=1500.0,
        min_alt=0.0, max_alt=1000.0,
        max_closing_speed=80.0,
    )
    assert env.contains(range_m=500.0, alt_m=200.0) is True


def test_engagement_envelope_out_of_range_too_far():
    env = EngagementEnvelope(100.0, 1500.0, 0.0, 1000.0, 80.0)
    assert env.contains(range_m=2000.0, alt_m=200.0) is False


def test_engagement_envelope_out_of_range_too_low_alt():
    env = EngagementEnvelope(100.0, 1500.0, 50.0, 1000.0, 80.0)
    assert env.contains(range_m=500.0, alt_m=10.0) is False


def test_track_distance_to_origin():
    t = Track(
        track_id=7,
        position=(30.0, 40.0, 0.0),
        velocity=(0.0, 0.0, 0.0),
        threat_score=0.5,
        state="DETECTED",
    )
    assert math.isclose(t.range_from(0.0, 0.0, 0.0), 50.0)


def test_weapon_state_marks_unavailable_when_not_idle():
    s = WeaponState(
        weapon_id="anvil_1",
        weapon_type="kinetic_quad",
        position=(0.0, 0.0, 0.0),
        available=False,
        ammo_remaining=None,
    )
    assert s.available is False
    assert s.ammo_remaining is None
