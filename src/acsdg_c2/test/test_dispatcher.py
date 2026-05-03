"""Tests for the Dispatcher translator (weapon + track → EngagementOrder).

Phase 3 note: interceptor_id is now FLEET-driven (not numeric-tail). All weapon_id
strings used here must exist in fleet.FLEET. Slots at time of writing:
  coyote_0 → 1,  anvil_1 → 2,  anvil_2 → 3,  anvil_3 → 4.
"""

import pytest

from acsdg_c2.dispatcher import Dispatcher
from acsdg_c2.weapons.anvil import Anvil
from acsdg_c2.weapons.types import Track


def make_track(track_id=7, pos=(500.0, 0.0, 50.0), threat=0.6):
    return Track(track_id=track_id, position=pos, velocity=(0.0, 0.0, 0.0),
                 threat_score=threat, state="DETECTED")


def test_dispatcher_translates_anvil_dispatch_to_engagement_order_payload():
    """anvil_1 is FLEET slot 2; weapon_id → interceptor_id comes from FLEET."""
    d = Dispatcher()
    a = Anvil("anvil_1", (-177.0, 177.0, 20.0))
    order = d.translate(a, make_track(track_id=7, threat=0.5))
    assert order["target_id"] == 7
    assert order["interceptor_id"] == 2     # FLEET: anvil_1 → interceptor_id=2
    assert order["priority"] == 127         # int(0.5 * 255) = 127


def test_dispatcher_handles_anvil_2_maps_to_slot_3():
    """anvil_2 is FLEET slot 3 (NW post)."""
    d = Dispatcher()
    a = Anvil("anvil_2", (177.0, -177.0, 20.0))
    order = d.translate(a, make_track())
    assert order["interceptor_id"] == 3


def test_dispatcher_priority_clamps_to_uint8_range():
    d = Dispatcher()
    a = Anvil("anvil_1", (-177.0, 177.0, 20.0))
    order = d.translate(a, make_track(threat=2.0))    # > 1.0 corner case
    assert 0 <= order["priority"] <= 255


def test_dispatcher_marks_weapon_engaged():
    d = Dispatcher()
    a = Anvil("anvil_1", (-177.0, 177.0, 20.0))
    assert a.is_available() is True
    d.translate(a, make_track(track_id=11))
    assert a.is_available() is False


def test_dispatcher_raises_for_unknown_weapon_id():
    """A weapon_id not in FLEET must raise ValueError, not silently return 0."""
    d = Dispatcher()
    a = Anvil("anvil_0", (0.0, 0.0, 20.0))   # anvil_0 is not in FLEET
    with pytest.raises(ValueError, match="not found in FLEET"):
        d.translate(a, make_track())


def test_dispatcher_routes_dronehunter_to_slot_3():
    """Phase 3 prep regression: weapon_id='dronehunter_0' must route to
    interceptor_id=3 (the SE post per FLEET), not interceptor_id=1 (the old
    numeric-tail logic would have given 0+1=1, colliding with Coyote)."""
    from acsdg_c2.fleet import FLEET
    matches = [s for s in FLEET if s.weapon_id == 'dronehunter_0']
    if not matches:
        pytest.skip("dronehunter_0 not in FLEET yet (added in Task 5)")
    assert matches[0].interceptor_id == 3
