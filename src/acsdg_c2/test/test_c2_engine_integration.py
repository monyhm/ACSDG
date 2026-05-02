"""Integration test for C2EngineNode — composes weapons / classifier / cost_function / assignment / dispatcher."""

import json
from dataclasses import dataclass, field
from typing import List

import pytest
import rclpy
from std_msgs.msg import Bool, String
from acsdg_msgs.msg import EngagementOrder, InterceptorState

from acsdg_c2.c2_engine_node import C2EngineNode


@dataclass
class _FakePub:
    msgs: List = field(default_factory=list)

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope="module")
def rclpy_runtime():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


def _make_interceptor_state(iid: int, x: float, y: float, z: float,
                            status: str = "IDLE", target_id: int = 0) -> InterceptorState:
    msg = InterceptorState()
    msg.id = iid
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.status = status
    msg.target_id = target_id
    return msg


def _make_targets_msg(payload: list) -> String:
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def test_c2_engine_emits_orders_for_fused_targets(rclpy_runtime):
    """End-to-end composition: fed fusion + interceptor state, emits orders."""
    node = C2EngineNode()
    engage_pub = _FakePub()
    scores_pub = _FakePub()
    node._engage_pub = engage_pub
    node._scores_pub = scores_pub

    # Arm the mission
    wave_msg = Bool()
    wave_msg.data = True
    node._on_wave(wave_msg)

    # Populate interceptor positions (4 interceptors at corner posts, all IDLE)
    homes = [(-100.0, -100.0, 20.0), (100.0, -100.0, 20.0),
             (100.0, 100.0, 20.0), (-100.0, 100.0, 20.0)]
    for iid, (x, y, z) in enumerate(homes, start=1):
        node._on_interceptor_state(_make_interceptor_state(iid, x, y, z))

    # Feed two fused targets — both within Anvil envelope (1.5 km)
    targets_payload = [
        {"id": 1,
         "position": {"x": 200.0, "y": 0.0, "z": 50.0},
         "velocity": {"x": -5.0, "y": 0.0, "z": 0.0},
         "state": "DETECTED",
         "threat_score": 0.0,
         "stamp": {"sec": 0, "nanosec": 0}},
        {"id": 2,
         "position": {"x": -200.0, "y": 100.0, "z": 50.0},
         "velocity": {"x": 5.0, "y": 0.0, "z": 0.0},
         "state": "DETECTED",
         "threat_score": 0.0,
         "stamp": {"sec": 0, "nanosec": 0}},
    ]
    node._on_targets(_make_targets_msg(targets_payload))

    # Run one timer tick
    node._on_timer()

    # Assertions
    assert len(scores_pub.msgs) >= 1, "expected at least one threat-score publication"
    score_payload = json.loads(scores_pub.msgs[-1].data)
    assert isinstance(score_payload, list)
    assert len(score_payload) == 2
    assert {entry["id"] for entry in score_payload} == {1, 2}

    assert len(engage_pub.msgs) >= 1, "expected at least one engagement order"
    for order in engage_pub.msgs:
        assert isinstance(order, EngagementOrder)
        assert order.target_id in {1, 2}
        assert order.interceptor_id in {1, 2, 3, 4}
        assert 0 <= order.priority <= 255

    node.destroy_node()


def test_c2_engine_does_not_emit_orders_before_wave_armed(rclpy_runtime):
    """Mission gating: no orders until /mission/wave_trigger arrives."""
    node = C2EngineNode()
    engage_pub = _FakePub()
    scores_pub = _FakePub()
    node._engage_pub = engage_pub
    node._scores_pub = scores_pub

    # Skip the wave trigger.
    homes = [(-100.0, -100.0, 20.0), (100.0, -100.0, 20.0),
             (100.0, 100.0, 20.0), (-100.0, 100.0, 20.0)]
    for iid, (x, y, z) in enumerate(homes, start=1):
        node._on_interceptor_state(_make_interceptor_state(iid, x, y, z))

    targets_payload = [
        {"id": 1,
         "position": {"x": 200.0, "y": 0.0, "z": 50.0},
         "velocity": {"x": -5.0, "y": 0.0, "z": 0.0},
         "state": "DETECTED",
         "threat_score": 0.0,
         "stamp": {"sec": 0, "nanosec": 0}},
    ]
    node._on_targets(_make_targets_msg(targets_payload))

    node._on_timer()

    assert len(engage_pub.msgs) == 0, "expected no orders before wave armed"

    node.destroy_node()
