"""Regression tests for the FLEET single-source-of-truth module."""
import pytest
from pathlib import Path

from acsdg_c2.fleet import FLEET, Slot, _validate_fleet, assert_matches_sdf, bridged_models, parse_sdf_includes
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


def _real_sdf_path() -> Path:
    # Test runs from the workspace root (colcon test) or from the package
    # root (direct pytest). Walk up until we find acsdg_gazebo.
    here = Path(__file__).resolve()
    for parent in (here, *here.parents):
        candidate = parent / "src" / "acsdg_gazebo" / "worlds" / "military_base.sdf"
        if candidate.is_file():
            return candidate
        candidate = parent / "acsdg_gazebo" / "worlds" / "military_base.sdf"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("Could not locate military_base.sdf from test working dir")


def test_parse_sdf_includes_returns_model_to_pose_dict():
    sdf = _real_sdf_path()
    poses = parse_sdf_includes(str(sdf))
    # Every FLEET model name must appear in the parsed SDF
    for slot in FLEET:
        name = f"{slot.gz_model_kind}_{slot.gz_instance_index}"
        assert name in poses, f"{name} not found in SDF includes"
    # Pose tuples are (x, y, z)
    for name, xyz in poses.items():
        assert isinstance(xyz, tuple) and len(xyz) == 3
        for v in xyz:
            assert isinstance(v, float)


def test_fleet_matches_sdf_spawn_poses():
    """Every slot's home must match the SDF's <include><pose> for that model name."""
    assert_matches_sdf(str(_real_sdf_path()))   # raises on disagreement


def test_assert_matches_sdf_raises_on_disagreement(tmp_path):
    """Synthetic SDF with a wrong pose must fail the assertion."""
    bad_sdf = tmp_path / "bad.sdf"
    bad_sdf.write_text(
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9"><world name="w">\n'
        '  <include><name>coyote_1</name>'
        '    <pose>0 0 0 0 0 0</pose>'  # wrong; FLEET says (177,177,20)
        '    <uri>foo</uri></include>\n'
        '  <include><name>interceptor_2</name>'
        '    <pose>-177 177 20 0 0 0</pose><uri>foo</uri></include>\n'
        '  <include><name>interceptor_3</name>'
        '    <pose>177 -177 20 0 0 0</pose><uri>foo</uri></include>\n'
        '  <include><name>interceptor_4</name>'
        '    <pose>-177 -177 20 0 0 0</pose><uri>foo</uri></include>\n'
        '</world></sdf>\n'
    )
    with pytest.raises(AssertionError, match="coyote_1"):
        assert_matches_sdf(str(bad_sdf))


def test_bridged_models_returns_kind_and_max_instance_index():
    pairs = bridged_models()
    pair_dict = dict(pairs)
    # Every kind in FLEET must appear with index ≥ max(gz_instance_index for that kind)
    expected: dict[str, int] = {}
    for slot in FLEET:
        expected[slot.gz_model_kind] = max(
            expected.get(slot.gz_model_kind, 0), slot.gz_instance_index)
    for kind, max_idx in expected.items():
        assert pair_dict[kind] == max_idx
    # Pairs are sorted alphabetically by kind for stable bridge wiring
    assert list(pairs) == sorted(pairs)


def test_bridged_models_for_phase3_inventory():
    """Phase 3 FLEET = 1 Coyote (gz instance 1) + 1 DroneHunter (gz instance 1)
    + 2 Anvils (gz instances 2 and 4) → bridged_models returns the max instance
    index per kind, sorted alphabetically."""
    assert bridged_models() == (('coyote', 1), ('dronehunter', 1), ('interceptor', 4))
