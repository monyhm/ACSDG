"""Tests for DroneHunter F700 weapon class."""
import pytest

from acsdg_c2.weapons import DroneHunter, Track
from acsdg_c2.weapons.types import TargetClass


def test_dronehunter_envelope_matches_spec():
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    env = dh.engagement_envelope()
    assert env.min_range == 50.0
    assert env.max_range == 2000.0
    assert env.max_alt == 4000.0


def test_dronehunter_pkill_table_matches_spec():
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    assert dh.pkill(TargetClass.SMALL_QUAD)        == 0.85
    assert dh.pkill(TargetClass.GROUP_1_FIXED_WING) == 0.70
    assert dh.pkill(TargetClass.GROUP_3_LOITERING)  == 0.50
    assert dh.pkill(TargetClass.SHAHED_CLASS)       == 0.40


def test_dronehunter_resource_cost():
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    assert dh.resource_cost() == 0.20


def test_dronehunter_time_to_intercept_credits_inbound_closing_rate():
    """Closing target at 30 m/s should yield ToI smaller than the static
    range/max_speed estimate. Mirrors the Coyote/Anvil closing-rate test."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    closing = Track(track_id=1, position=(500.0, 0.0, 20.0),
                    velocity=(-30.0, 0.0, 0.0),  # closing at 30 m/s
                    threat_score=0.0, state="DETECTED")
    toi = dh.time_to_intercept(closing)
    static_toi = 500.0 / 31.0  # ≈ 16.1 s
    assert toi < static_toi


def test_dronehunter_time_to_intercept_clamps_eff_speed_for_fleeing_shahed():
    """Fleeing SHAHED at 185 m/s vs DroneHunter's 31 m/s — eff_speed clamps
    at 0.5*max_speed = 15.5 m/s. ToI = 1000/15.5 ≈ 64.5s. Regression guard."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    fleeing = Track(track_id=1, position=(1000.0, 0.0, 20.0),
                    velocity=(185.0, 0.0, 0.0),
                    threat_score=0.0, state="DETECTED")
    toi = dh.time_to_intercept(fleeing)
    assert toi == pytest.approx(64.5, rel=0.05)
    assert toi < 200.0   # regression guard against eff_speed=1.0 floor


def test_dronehunter_is_available_returns_false_during_cooldown():
    """After mark_idle on a recently-engaged DroneHunter, is_available is
    False until 180 s elapse — Phase-3 multi-shot relaunch_time."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    assert dh.is_available()  # idle, no prior engagement
    dh.mark_engaged(target_id=42)
    assert not dh.is_available()
    dh.mark_idle()  # arrival home → cooldown starts
    assert not dh.is_available()  # cooldown active


def test_dronehunter_cooldown_clears_after_relaunch_time(monkeypatch):
    """is_available() returns True once 180 s have elapsed since mark_idle.
    Use monkeypatch on time.time to advance the clock without sleeping."""
    import acsdg_c2.weapons.dronehunter as dh_mod

    fake_now = [1000.0]
    monkeypatch.setattr(dh_mod, "time", type("T", (), {"time": lambda: fake_now[0]}))

    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    dh.mark_engaged(target_id=42)
    dh.mark_idle()
    assert not dh.is_available()  # cooldown started at fake_now=1000

    fake_now[0] = 1000.0 + 179.99   # one tick before expiry
    assert not dh.is_available()

    fake_now[0] = 1000.0 + 180.01   # cooldown expired
    assert dh.is_available()


def test_dronehunter_mark_idle_without_prior_engagement_does_not_start_cooldown():
    """The was_engaged guard: mark_idle ticks while idle (10 Hz) must not
    reset the cooldown clock. Otherwise the cooldown would never expire."""
    dh = DroneHunter(weapon_id="dh_test", home_position=(0.0, 0.0, 20.0))
    dh.mark_idle()  # was already idle
    dh.mark_idle()  # another tick — must not start cooldown
    assert dh.is_available()
