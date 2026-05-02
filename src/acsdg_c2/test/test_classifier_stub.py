"""Tests for the Phase-1 classifier stub."""

import pytest

from acsdg_c2.classifier import Classifier
from acsdg_c2.weapons.types import TargetClass, Track


def make_track(track_id=1, pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)):
    return Track(track_id=track_id, position=pos, velocity=vel,
                 threat_score=0.0, state="DETECTED")


def test_phase1_posterior_returns_full_mass_on_small_quad():
    c = Classifier()
    posterior = c.posterior(make_track())
    assert posterior[TargetClass.SMALL_QUAD] == pytest.approx(1.0)
    for cls in TargetClass:
        if cls is not TargetClass.SMALL_QUAD:
            assert posterior[cls] == pytest.approx(0.0)


def test_posterior_sums_to_one_for_any_track():
    c = Classifier()
    p = c.posterior(make_track(track_id=42, pos=(100.0, 200.0, 50.0)))
    assert sum(p.values()) == pytest.approx(1.0)


def test_posterior_keys_are_all_four_target_classes():
    c = Classifier()
    p = c.posterior(make_track())
    assert set(p.keys()) == set(TargetClass)
