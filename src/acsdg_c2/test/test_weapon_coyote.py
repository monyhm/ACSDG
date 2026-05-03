"""Tests for Coyote — the Phase-2 frag-jet weapon system."""

import pytest

from acsdg_c2.weapons.types import TargetClass, Track
from acsdg_c2.weapons.coyote import Coyote


def make_coyote(weapon_id: str = "coyote_0",
                home: tuple = (177.0, 177.0, 20.0)) -> Coyote:
    return Coyote(weapon_id=weapon_id, home_position=home)


def make_track(track_id: int = 1, pos: tuple = (300.0, 0.0, 50.0),
               vel: tuple = (-5.0, 0.0, 0.0)) -> Track:
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=0.7, state="DETECTED")


def test_coyote_weapon_type_is_frag_jet():
    c = make_coyote()
    assert c.weapon_type == "frag_jet"


def test_coyote_pkill_table_matches_spec():
    c = make_coyote()
    # spec §6.2 — frag wasted on tiny targets, optimal on Group 3 / Shahed
    assert c.pkill(TargetClass.SMALL_QUAD) == pytest.approx(0.60)
    assert c.pkill(TargetClass.GROUP_1_FIXED_WING) == pytest.approx(0.85)
    assert c.pkill(TargetClass.GROUP_3_LOITERING) == pytest.approx(0.95)
    assert c.pkill(TargetClass.SHAHED_CLASS) == pytest.approx(0.95)


def test_coyote_engagement_envelope_matches_spec():
    c = make_coyote()
    env = c.engagement_envelope()
    assert env.min_range == pytest.approx(100.0)    # booster-clear from launcher
    assert env.max_range == pytest.approx(5000.0)   # 5 km in sim, spec §6.2
    assert env.min_alt == pytest.approx(0.0)
    assert env.max_alt == pytest.approx(4500.0)     # Group-3 loitering ceiling, §7
    assert env.max_closing_speed == pytest.approx(160.0)


def test_coyote_time_to_intercept_credits_inbound_closing_rate():
    """Closing-rate code path: target inbound velocity should add to eff_speed."""
    c = make_coyote(home=(0.0, 0.0, 0.0))
    # Target 1500 m away in +x, moving at -10 m/s (inbound along LOS).
    # eff_speed = max_speed (160) + closing (10) = 170; ToI = 1500 / 170 s.
    track = make_track(pos=(1500.0, 0.0, 0.0), vel=(-10.0, 0.0, 0.0))
    assert c.time_to_intercept(track) == pytest.approx(1500.0 / 170.0, rel=0.01)


def test_coyote_can_engage_far_target_within_envelope():
    c = make_coyote(home=(177.0, 177.0, 20.0))
    track = make_track(pos=(177.0, -2000.0, 50.0))   # ~2.2 km away — fine for Coyote
    assert c.can_engage(track) is True


def test_coyote_cannot_engage_out_of_range():
    c = make_coyote(home=(177.0, 177.0, 20.0))
    track = make_track(pos=(7000.0, 0.0, 50.0))   # >5 km
    assert c.can_engage(track) is False


def test_coyote_cannot_engage_when_unavailable():
    c = make_coyote()
    c.mark_engaged(target_id=42)
    track = make_track(pos=(500.0, 0.0, 50.0))
    assert c.is_available() is False
    assert c.can_engage(track) is False


def test_coyote_dispatch_records_engagement_and_returns_payload():
    c = make_coyote()
    track = make_track(track_id=99, pos=(500.0, 0.0, 50.0))
    payload = c.dispatch(track)
    assert payload["target_id"] == 99
    assert payload["weapon_id"] == "coyote_0"
    assert c.is_available() is False
    assert c.engaged_target_id() == 99


def test_coyote_engaged_target_id_resets_on_mark_idle():
    c = make_coyote()
    c.mark_engaged(target_id=7)
    assert c.engaged_target_id() == 7
    c.mark_idle()
    assert c.engaged_target_id() is None


def test_coyote_resource_cost_matches_spec():
    c = make_coyote()
    assert c.resource_cost() == pytest.approx(0.10)


def test_coyote_time_to_intercept_uses_max_speed_and_closing_rate():
    c = make_coyote(home=(0.0, 0.0, 0.0))
    # Target 1600 m away, closing at 0 along the line of sight.
    # eff_speed = max_speed (160) + 0 = 160; ToI = 1600 / 160 = 10 s.
    track = make_track(pos=(1600.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0))
    assert c.time_to_intercept(track) == pytest.approx(10.0, rel=0.01)


def test_coyote_state_marks_one_shot():
    c = make_coyote()
    s = c.state()
    assert s.weapon_id == "coyote_0"
    assert s.weapon_type == "frag_jet"
    assert s.ammo_remaining is None   # one-shot, like Anvil


def test_coyote_time_to_intercept_clamps_eff_speed_for_stern_shahed():
    """SHAHED at 185 m/s tail-chase against 160 m/s Coyote — eff_speed should clamp at 80 m/s,
    not the historical 1.0 m/s blow-up. Closes N-C."""
    coyote = Coyote(weapon_id="coyote_test", home_position=(0.0, 0.0, 20.0))
    shahed = Track(
        track_id=1,
        position=(1000.0, 0.0, 50.0),
        velocity=(185.0, 0.0, 0.0),
        threat_score=0.0, state="DETECTED",
    )
    toi = coyote.time_to_intercept(shahed)
    # Expected ≈ 1000 / (0.5 * 160) = 1000 / 80 = 12.5 s
    assert 11.5 < toi < 14.0, f"Got {toi}, expected ~12.5 s"
    # Regression check: original buggy code returned 1000.0 (range / 1.0)
    assert toi < 50.0, f"Got {toi} — eff_speed=1.0 floor regression"
