"""Bayesian classifier — Phase 1 stub.

The Phase-1 stub returns a degenerate posterior with all mass on SMALL_QUAD.
Phase 4 replaces the body with a real naive-Bayes Gaussian classifier driven
by per-track size/speed/RCS/IR observations. The interface stays the same.
"""

from __future__ import annotations

from typing import Dict

from acsdg_c2.weapons.types import TargetClass, Track


class Classifier:

    def posterior(self, track: Track) -> Dict[TargetClass, float]:
        """Return P(class | observations) for each TargetClass.

        Phase 1: deterministic — full mass on SMALL_QUAD.
        """
        return {
            TargetClass.SMALL_QUAD: 1.0,
            TargetClass.GROUP_1_FIXED_WING: 0.0,
            TargetClass.GROUP_3_LOITERING: 0.0,
            TargetClass.SHAHED_CLASS: 0.0,
        }
