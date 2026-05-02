#!/usr/bin/env python3
"""
gz_bridge_shim.py — ROS 2 Humble ↔ Gazebo Harmonic data-plane bridge.

The stock `ros_gz_bridge` that ships with ROS 2 Humble is built against
libignition-transport11 / libignition-msgs8 (Ignition Fortress era). The
Gazebo Harmonic simulator uses gz.transport13 / gz.msgs10. Type strings
differ between the two namespaces, so gz-transport silently refuses to
deliver messages across them — cmd_vel never reaches the drone, and
odometry never reaches ROS.

This shim re-bridges the data plane using the gz.transport13 / gz.msgs10
Python bindings directly, which ARE installed on the system (alongside
Gazebo Harmonic). It handles cmd_vel and odometry for every model kind in
the `_BRIDGED_MODELS` table below — currently enemy + interceptor + coyote.

Per-kind, per-instance topic shape:
  ROS  Twist     /{kind}_{N}/cmd_vel        → gz.msgs.Twist     /model/{kind}_{N}/cmd_vel
  gz.msgs.Odom   /model/{kind}_{N}/odometry → ROS Odometry      /model/{kind}_{N}/odometry
"""

import math
import os
import sys
import threading

# Make the sibling bridge_math module importable regardless of how this
# script is launched (directly, via colcon symlink-install, or as the ROS 2
# entry point). Both files install into lib/${PROJECT_NAME}/ side-by-side.
_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.msgs10.odometry_pb2 import Odometry as GzOdometry

from bridge_math import world_to_body_velocity


# Models bridged on cmd_vel + odometry, as (kind, count) tuples. The bridge
# wires `/{kind}_{1..count}/cmd_vel` (ROS) ↔ `/model/{kind}_{i}/cmd_vel` (gz)
# for every kind. Phase 1 had only enemy + interceptor; Phase 2 replaces
# interceptor_1 with coyote_1 at the NE post.
#
# interceptor stays at count=4 (not 3) because the table is `count`-based,
# not instance-list-based. Dropping to 3 would mis-wire the SURVIVING
# interceptors at slots 2..4 (the loop walks range(1, count+1), so count=3
# would wire instances 1,2,3 instead of 2,3,4). Keeping count=4 wires a
# stale slot for instance 1, which is harmless: gz transport advertise/
# subscribe on a topic with no peers just keeps an unused slot — no error,
# no warning, no resource leak. If a future call site moves to explicit
# instance lists `('interceptor', (2, 3, 4))` we can drop it then.
_BRIDGED_MODELS = (
    ('enemy',       4),    # enemy_1..4 — Anvils' targets
    ('interceptor', 4),    # interceptor_1..4 — slot 1 unused after Phase 2 (see note above)
    ('coyote',      1),    # coyote_1 — new in Phase 2 at slot 1 (NE post)
)


class BridgeShim(Node):

    def __init__(self) -> None:
        super().__init__('gz_bridge_shim')
        self._gz = GzNode()
        self._gz_pubs:  dict = {}
        self._ros_pubs: dict = {}

        # Cached body orientation per model (qx, qy, qz, qw) from incoming
        # odometry. gz-sim-velocity-control-system applies cmd_vel in BODY
        # frame; controllers and enemy_driver publish in WORLD frame. We
        # use the FULL quaternion (not just yaw) so the rotation is correct
        # even when the body has rolled/pitched off-axis — which happens in
        # this sim because nothing physically constrains body orientation
        # despite cmd_vel.angular always being (0,0,0). With the full
        # rotation, body's center-of-mass translates correctly in world
        # regardless of whatever orientation physics has put it in.
        self._quat: dict = {}    # model -> (qx, qy, qz, qw)
        self._quat_lock = threading.Lock()

        # Per-drone bindings driven by the _BRIDGED_MODELS table.
        for kind, count in _BRIDGED_MODELS:
            self._wire_twist_ros_to_gz(kind, count)
            self._wire_odom_gz_to_ros(kind, count)

        self.get_logger().info(
            'BridgeShim ready — bridged: '
            + ', '.join(f'{count} × {kind}'
                        for kind, count in _BRIDGED_MODELS))

    # ── Twist: ROS → Gazebo ─────────────────────────────────────────────

    def _wire_twist_ros_to_gz(self, kind: str, count: int) -> None:
        for i in range(1, count + 1):
            ros_topic = f'/{kind}_{i}/cmd_vel'
            gz_topic  = f'/model/{kind}_{i}/cmd_vel'
            model     = f'{kind}_{i}'
            gz_pub = self._gz.advertise(gz_topic, GzTwist)
            if not gz_pub:
                self.get_logger().error(f'Failed to advertise gz topic {gz_topic}')
                continue
            self._gz_pubs[gz_topic] = gz_pub
            self.create_subscription(
                Twist, ros_topic,
                lambda msg, t=gz_topic, m=model: self._forward_twist(msg, t, m), 10)

    def _forward_twist(self, msg: Twist, gz_topic: str, model: str) -> None:
        # gz-sim-velocity-control-system applies cmd_vel in BODY frame
        # (verified against gz-sim8 source — no world-frame SDF option,
        # plugin writes LinearVelocityCmd directly without any rotation).
        # The controller / enemy_driver publish in WORLD frame, so this shim
        # must inverse-rotate by the body's current orientation before
        # forwarding. Using the full quaternion (not just yaw) means the
        # transform stays correct even if the body has rolled or pitched —
        # which happens in this sim because nothing physically clamps body
        # orientation, despite cmd_vel.angular always being (0,0,0).
        #
        # We also force angular to zero on the forwarded twist as a defense
        # in depth: the body should NEVER receive a non-zero angular cmd in
        # this project (drones translate without yawing/pitching), and
        # zeroing here protects against any future caller mistakenly
        # publishing one.
        with self._quat_lock:
            qx, qy, qz, qw = self._quat.get(model, (0.0, 0.0, 0.0, 1.0))

        bx, by, bz = world_to_body_velocity(
            qx, qy, qz, qw,
            msg.linear.x, msg.linear.y, msg.linear.z,
        )

        gz = GzTwist()
        gz.linear.x  = bx
        gz.linear.y  = by
        gz.linear.z  = bz
        gz.angular.x = 0.0
        gz.angular.y = 0.0
        gz.angular.z = 0.0
        self._gz_pubs[gz_topic].publish(gz)

    # ── Odometry: Gazebo → ROS ─────────────────────────────────────────

    def _wire_odom_gz_to_ros(self, kind: str, count: int) -> None:
        for i in range(1, count + 1):
            topic = f'/model/{kind}_{i}/odometry'
            model = f'{kind}_{i}'
            self._ros_pubs[topic] = self.create_publisher(Odometry, topic, 10)
            ok = self._gz.subscribe(
                GzOdometry, topic,
                lambda msg, t=topic, m=model: self._forward_odom(msg, t, m))
            if not ok:
                self.get_logger().error(f'Failed to subscribe gz topic {topic}')

    def _forward_odom(self, gz: GzOdometry, ros_topic: str, model: str) -> None:
        # Cache the full orientation quaternion (not just yaw) for the
        # world→body rotation in _forward_twist. The simple yaw extraction
        # `2*atan2(qz, qw)` only works when roll and pitch are ≈0 — which
        # they aren't in practice here (numerical drift puts non-zero
        # roll/pitch on the bodies despite zero commanded angular velocity).
        with self._quat_lock:
            self._quat[model] = (
                gz.pose.orientation.x,
                gz.pose.orientation.y,
                gz.pose.orientation.z,
                gz.pose.orientation.w,
            )
        ros_msg = Odometry()
        ros_msg.header.stamp    = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = 'world'
        # Pose
        ros_msg.pose.pose.position.x    = gz.pose.position.x
        ros_msg.pose.pose.position.y    = gz.pose.position.y
        ros_msg.pose.pose.position.z    = gz.pose.position.z
        ros_msg.pose.pose.orientation.x = gz.pose.orientation.x
        ros_msg.pose.pose.orientation.y = gz.pose.orientation.y
        ros_msg.pose.pose.orientation.z = gz.pose.orientation.z
        ros_msg.pose.pose.orientation.w = gz.pose.orientation.w
        # Twist
        ros_msg.twist.twist.linear.x  = gz.twist.linear.x
        ros_msg.twist.twist.linear.y  = gz.twist.linear.y
        ros_msg.twist.twist.linear.z  = gz.twist.linear.z
        ros_msg.twist.twist.angular.x = gz.twist.angular.x
        ros_msg.twist.twist.angular.y = gz.twist.angular.y
        ros_msg.twist.twist.angular.z = gz.twist.angular.z
        self._ros_pubs[ros_topic].publish(ros_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
