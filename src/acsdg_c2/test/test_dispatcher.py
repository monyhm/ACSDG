"""Tests for the Dispatcher translator (weapon + track → EngagementOrder)."""

import pytest

from acsdg_c2.dispatcher import Dispatcher
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.types import Track


def make_track(track_id=7, pos=(500.0, 0.0, 50.0), threat=0.6):
    return Track(track_id=track_id, position=pos, velocity=(0.0, 0.0, 0.0),
                 threat_score=threat, state="DETECTED")


def test_dispatcher_translates_anvil_dispatch_to_engagement_order_payload():
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))
    order = d.translate(a, make_track(track_id=7, threat=0.5))
    assert order["target_id"] == 7
    assert order["interceptor_id"] == 1     # "anvil_0" → 1 (numeric tail + 1)
    assert order["priority"] == 127         # round(0.5 * 255) = 127 or 128


def test_dispatcher_handles_anvil_3_index():
    d = Dispatcher()
    a = Anvil("anvil_2", (0.0, 0.0, 20.0))
    order = d.translate(a, make_track())
    assert order["interceptor_id"] == 3


def test_dispatcher_priority_clamps_to_uint8_range():
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))
    order = d.translate(a, make_track(threat=2.0))    # > 1.0 corner case
    assert 0 <= order["priority"] <= 255


def test_dispatcher_marks_weapon_engaged():
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))
    assert a.is_available() is True
    d.translate(a, make_track(track_id=11))
    assert a.is_available() is False
