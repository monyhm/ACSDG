"""Weapon abstractions for the ACSDG C2 layer.

Phase 1 only exports the domain types and the abstract base class;
later phases add concrete WeaponSystem subclasses (Anvil, Coyote, etc.).
"""

from acsdg_c2.weapons.types import (
    TargetClass,
    EngagementEnvelope,
    Track,
    WeaponState,
)

__all__ = ["TargetClass", "EngagementEnvelope", "Track", "WeaponState"]
