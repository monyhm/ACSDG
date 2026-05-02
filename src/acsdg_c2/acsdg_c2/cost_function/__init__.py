"""Cost-function module for C2 weapon-target pairing."""

from acsdg_c2.cost_function.threat_priority import score_threat
from acsdg_c2.cost_function.expected_utility import build_cost_matrix

__all__ = ["score_threat", "build_cost_matrix"]
