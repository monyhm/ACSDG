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
Gazebo Harmonic). It handles:

  ROS  Twist     /enemy_{N}/cmd_vel        → gz.msgs.Twist     /model/enemy_{N}/cmd_vel
  ROS  Twist     /interceptor_{N}/cmd_vel  → gz.msgs.Twist     /model/interceptor_{N}/cmd_vel
  gz.msgs.Odom   /model/enemy_{N}/odometry → ROS Odometry      /model/enemy_{N}/odometry
  gz.msgs.Odom   /model/interceptor_{N}/odometry → ROS Odometry /model/interceptor_{N}/odometry
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.msgs10.odometry_pb2 import Odometry as GzOdometry


ENEMY_COUNT       = 4
INTERCEPTOR_COUNT = 4


class BridgeShim(Node):

    def __init__(self) -> None:
        super().__init__('gz_bridge_shim')
        self._gz = GzNode()
        self._gz_pubs:  dict = {}
        self._ros_pubs: dict = {}

        # Cached yaw per model name (from incoming odometry) — used to
        # convert world-frame ROS Twist into body-frame for Gazebo's
        # VelocityControl plugin.  Enemies spawn with yaw=π (1,2) or 0
        # (3,4); interceptors spawn with yaw=0.  Without this rotation,
        # enemies 1 and 2 fly in the wrong direction.
        self._yaw: dict = {}
        self._yaw_lock = threading.Lock()

        # Per-drone bindings for both enemies and interceptors
        self._wire_twist_ros_to_gz('enemy',       ENEMY_COUNT)
        self._wire_twist_ros_to_gz('interceptor', INTERCEPTOR_COUNT)
        self._wire_odom_gz_to_ros('enemy',        ENEMY_COUNT)
        self._wire_odom_gz_to_ros('interceptor',  INTERCEPTOR_COUNT)

        self.get_logger().info(
            f'BridgeShim ready — {ENEMY_COUNT} enemies + {INTERCEPTOR_COUNT} '
            f'interceptors, both cmd_vel and odometry bridged.')

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
        # gz-sim-velocity-control-system applies cmd_vel in WORLD frame in
        # Harmonic — no rotation needed. Earlier versions of this shim
        # rotated by cached yaw assuming body-frame; once the body started
        # tumbling (no rotational damping with gravity off), that rotation
        # became actively wrong and interceptors drifted to z=2km.
        gz = GzTwist()
        gz.linear.x  = msg.linear.x
        gz.linear.y  = msg.linear.y
        gz.linear.z  = msg.linear.z
        gz.angular.x = msg.angular.x
        gz.angular.y = msg.angular.y
        gz.angular.z = msg.angular.z
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
        # Cache yaw for the world→body rotation used by _forward_twist.
        # yaw = 2 * atan2(qz, qw) when roll and pitch are ≈0.
        with self._yaw_lock:
            self._yaw[model] = 2.0 * math.atan2(
                gz.pose.orientation.z, gz.pose.orientation.w)
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
