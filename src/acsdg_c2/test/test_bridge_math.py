"""Regression tests for the gz_bridge_shim rotation math.

The bridge's `world_to_body_velocity` function is load-bearing: a previous
session removed it under a wrong belief, which silently broke enemy motion
for ~10 days. These tests pin the rotation behavior so it can't regress
again without a test failure.

Lives in the acsdg_c2 test tree so it picks up the existing pytest setup;
imports the helper from the acsdg_gazebo package via path adjustment.
"""

import math
import os
import sys

import pytest

# bridge_math.py lives in acsdg_gazebo/scripts/, two levels up from this
# test file plus across to the sibling package.
_BRIDGE_DIR = os.path.realpath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'acsdg_gazebo', 'scripts'))
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

from bridge_math import world_to_body_velocity  # noqa: E402


def _close(actual, expected, tol=1e-9):
    return all(abs(a - e) < tol for a, e in zip(actual, expected))


def test_identity_quaternion_is_passthrough():
    """qw=1, qx=qy=qz=0 means body and world are aligned; velocity unchanged."""
    bx, by, bz = world_to_body_velocity(0.0, 0.0, 0.0, 1.0, 1.5, 2.5, 3.5)
    assert _close((bx, by, bz), (1.5, 2.5, 3.5))


def test_yaw_pi_negates_x_and_y():
    """Body rotated 180° about z (qz=1, qw=0). x and y in body frame are world's -x, -y."""
    # World velocity (1, 2, 3) → body velocity (-1, -2, 3).
    bx, by, bz = world_to_body_velocity(0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0)
    assert _close((bx, by, bz), (-1.0, -2.0, 3.0))


def test_yaw_half_pi_swaps_x_and_y():
    """Body rotated +90° about z. World x-axis becomes body's -y; world y-axis becomes body's +x."""
    # quat for yaw=π/2 (CCW) is (0, 0, sin(π/4), cos(π/4))
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    bx, by, bz = world_to_body_velocity(0.0, 0.0, s, c, 1.0, 0.0, 0.0)
    # world x = (1,0,0) in world frame becomes (0, -1, 0) in body frame
    assert _close((bx, by, bz), (0.0, -1.0, 0.0), tol=1e-9)


def test_pitch_pi_inverts_x_and_z():
    """Body pitched 180° about y (qy=1, qw=0). World x and z become body's -x, -z."""
    bx, by, bz = world_to_body_velocity(0.0, 1.0, 0.0, 0.0, 1.0, 2.0, 3.0)
    assert _close((bx, by, bz), (-1.0, 2.0, -3.0))


def test_roll_pi_inverts_y_and_z():
    """Body rolled 180° about x (qx=1, qw=0). World y and z become body's -y, -z."""
    bx, by, bz = world_to_body_velocity(1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0)
    assert _close((bx, by, bz), (1.0, -2.0, -3.0))


def test_non_unit_quaternion_is_normalized_internally():
    """Defensive: scaled (non-unit) quaternion should produce same result as the unit form."""
    # Same yaw=π rotation, but scaled 5x — should still give (-1, -2, 3).
    bx, by, bz = world_to_body_velocity(0.0, 0.0, 5.0, 0.0, 1.0, 2.0, 3.0)
    assert _close((bx, by, bz), (-1.0, -2.0, 3.0))


def test_zero_quaternion_falls_back_to_identity():
    """Defensive: if a degenerate (~zero-norm) quat slips through, don't NaN."""
    bx, by, bz = world_to_body_velocity(0.0, 0.0, 0.0, 0.0, 1.5, 2.5, 3.5)
    assert _close((bx, by, bz), (1.5, 2.5, 3.5))


def test_arbitrary_unit_quaternion_round_trips():
    """For any unit q, transforming v_world to body and back recovers v_world."""
    # arbitrary unit quat (axis-angle: 30° around (1, 1, 1)/sqrt(3))
    angle = math.radians(30)
    axis = (1.0 / math.sqrt(3),) * 3
    s = math.sin(angle / 2)
    qx, qy, qz, qw = axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)

    vx, vy, vz = 4.0, -7.0, 2.5
    bx, by, bz = world_to_body_velocity(qx, qy, qz, qw, vx, vy, vz)

    # Apply forward rotation: world = R(q) * body. Forward rotation is the
    # transpose of inverse, so we re-use world_to_body_velocity with the
    # conjugate quaternion (which represents R(q)^-1 → its inverse-rotate
    # is R(q) itself).
    rx, ry, rz = world_to_body_velocity(-qx, -qy, -qz, qw, bx, by, bz)
    assert _close((rx, ry, rz), (vx, vy, vz), tol=1e-9)
