"""Regression tests for the FLEET single-source-of-truth module."""
import pytest

from acsdg_c2.fleet import FLEET, Slot, _validate_fleet
from acsdg_c2.weapons import Anvil


def test_fleet_has_four_slots():
    assert len(FLEET) == 4


def test_fleet_slots_are_frozen():
    """Slot must be immutable so consumers cannot mutate the canonical fleet."""
    s = FLEET[0]
    with pytest.raises((AttributeError, TypeError)):
        s.interceptor_id = 99  # type: ignore


def test_fleet_uniqueness_validator_rejects_duplicate_interceptor_id():
    bad = (
        Slot(1, Anvil, "a", "anvil_node", "interceptor", 1, (0.0, 0.0, 20.0)),
        Slot(1, Anvil, "b", "anvil_node", "interceptor", 2, (1.0, 0.0, 20.0)),
    )
    with pytest.raises(ValueError, match="duplicate interceptor_id"):
        _validate_fleet(bad)


def test_fleet_uniqueness_validator_rejects_duplicate_weapon_id():
    bad = (
        Slot(1, Anvil, "anvil_1", "anvil_node", "interceptor", 1, (0.0, 0.0, 20.0)),
        Slot(2, Anvil, "anvil_1", "anvil_node", "interceptor", 2, (1.0, 0.0, 20.0)),
    )
    with pytest.raises(ValueError, match="duplicate weapon_id"):
        _validate_fleet(bad)


def test_fleet_uniqueness_validator_rejects_duplicate_gz_model():
    bad = (
        Slot(1, Anvil, "a", "anvil_node", "interceptor", 2, (0.0, 0.0, 20.0)),
        Slot(2, Anvil, "b", "anvil_node", "interceptor", 2, (1.0, 0.0, 20.0)),
    )
    with pytest.raises(ValueError, match="duplicate gz model"):
        _validate_fleet(bad)


def test_fleet_uniqueness_validator_rejects_duplicate_home():
    bad = (
        Slot(1, Anvil, "a", "anvil_node", "interceptor", 1, (177.0, 177.0, 20.0)),
        Slot(2, Anvil, "b", "anvil_node", "interceptor", 2, (177.0, 177.0, 20.0)),
    )
    with pytest.raises(ValueError, match="duplicate home"):
        _validate_fleet(bad)


def test_fleet_validator_accepts_valid_fleet():
    """The shipping FLEET must pass validation (sanity check)."""
    _validate_fleet(FLEET)  # raises on failure
