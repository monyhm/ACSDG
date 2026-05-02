"""Tests for Anvil — the Phase-1 kinetic-quad weapon system."""

import pytest

from acsdg_c2.weapons.types import TargetClass, Track
from acsdg_c2.weapons.anvil import Anvil


def make_anvil(weapon_id: str = "anvil_0", home: tuple = (0.0, 0.0, 20.0)) -> Anvil:
    return Anvil(weapon_id=weapon_id, home_position=home)


def make_track(track_id: int = 1, pos: tuple = (100.0, 0.0, 50.0),
               vel: tuple = (-5.0, 0.0, 0.0)) -> Track:
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=0.7, state="DETECTED")


def test_anvil_weapon_type_is_kinetic_quad():
    a = make_anvil()
    assert a.weapon_type == "kinetic_quad"


def test_anvil_pkill_table_matches_spec():
    a = make_anvil()
    assert a.pkill(TargetClass.SMALL_QUAD) == pytest.approx(0.95)
    assert a.pkill(TargetClass.GROUP_1_FIXED_WING) == pytest.approx(0.85)
    assert a.pkill(TargetClass.GROUP_3_LOITERING) == pytest.approx(0.70)
    assert a.pkill(TargetClass.SHAHED_CLASS) == pytest.approx(0.40)


def test_anvil_engagement_envelope_matches_spec():
    a = make_anvil()
    env = a.engagement_envelope()
    assert env.max_range == pytest.approx(1500.0)   # 1.5 km from spec §6.1
    assert env.max_closing_speed == pytest.approx(45.0)   # m/s from spec §6.1


def test_anvil_can_engage_within_envelope_when_available():
    a = make_anvil(home=(0.0, 0.0, 20.0))
    track = make_track(pos=(500.0, 0.0, 50.0))   # 500 m horizontal, 30 m up
    assert a.can_engage(track) is True


def test_anvil_cannot_engage_out_of_range():
    a = make_anvil(home=(0.0, 0.0, 20.0))
    track = make_track(pos=(2000.0, 0.0, 50.0))  # > 1.5 km
    assert a.can_engage(track) is False


def test_anvil_cannot_engage_when_unavailable():
    a = make_anvil()
    a.mark_engaged(target_id=42)
    track = make_track(pos=(500.0, 0.0, 50.0))
    assert a.is_available() is False
    assert a.can_engage(track) is False


def test_anvil_dispatch_records_engagement_and_returns_order_payload():
    a = make_anvil()
    track = make_track(track_id=99, pos=(500.0, 0.0, 50.0))
    payload = a.dispatch(track)
    assert payload["target_id"] == 99
    assert payload["weapon_id"] == "anvil_0"
    assert a.is_available() is False
    assert a.engaged_target_id() == 99


def test_anvil_engaged_target_id_resets_on_mark_idle():
    a = make_anvil()
    a.mark_engaged(target_id=7)
    assert a.engaged_target_id() == 7
    a.mark_idle()
    assert a.engaged_target_id() is None


def test_anvil_resource_cost_matches_spec():
    a = make_anvil()
    assert a.resource_cost() == pytest.approx(0.05)


def test_anvil_time_to_intercept_uses_max_speed_and_closing_rate():
    a = make_anvil(home=(0.0, 0.0, 20.0))
    # Target 500 m away, closing at 5 m/s along the line of sight
    track = make_track(pos=(500.0, 0.0, 20.0), vel=(-5.0, 0.0, 0.0))
    # eff_speed = max_speed (45) + closing (5) = 50; t = 500 / 50 = 10 s
    assert a.time_to_intercept(track) == pytest.approx(10.0, rel=0.01)
