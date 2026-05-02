"""Pure-math helpers for the gz_bridge_shim.

Separated from gz_bridge_shim.py so the rotation math can be unit-tested
without a running ROS / Gazebo graph (this module imports only stdlib).
"""

import math
from typing import Tuple


def world_to_body_velocity(
    qx: float, qy: float, qz: float, qw: float,
    vx: float, vy: float, vz: float,
) -> Tuple[float, float, float]:
    """Inverse-rotate a world-frame velocity vector to body frame.

    Given a quaternion (qx, qy, qz, qw) representing the body's orientation
    relative to the world (Hamilton convention), returns R(q)^T · v_world.
    Used to convert a world-frame ROS Twist into the body-frame Gazebo
    cmd_vel that gz-sim-velocity-control-system expects.

    The quaternion is normalized internally as a defensive guard against
    odometry numerical drift. A degenerate (~zero-norm) quaternion falls
    back to the identity rotation rather than producing NaNs.
    """
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return vx, vy, vz
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    bx = ((1.0 - 2.0 * (qy * qy + qz * qz)) * vx
          + 2.0 * (qx * qy + qz * qw) * vy
          + 2.0 * (qx * qz - qy * qw) * vz)
    by = (2.0 * (qx * qy - qz * qw) * vx
          + (1.0 - 2.0 * (qx * qx + qz * qz)) * vy
          + 2.0 * (qy * qz + qx * qw) * vz)
    bz = (2.0 * (qx * qz + qy * qw) * vx
          + 2.0 * (qy * qz - qx * qw) * vy
          + (1.0 - 2.0 * (qx * qx + qy * qy)) * vz)
    return bx, by, bz
