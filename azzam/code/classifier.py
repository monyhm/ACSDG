"""Bayesian likelihood-ratio threat classifier.

Maps observed track features (RCS, speed, altitude) to posterior
probabilities over classification groups: MULTIROTOR, LOITERING,
FIXED_WING, UNKNOWN. Threat type means come from the locked spec
table. RCS σ is 30% of mean. Speed σ and altitude σ are per-type
(real flight-profile variance) so adjacent threat bands overlap and
off-spec speeds/altitudes don't fall into an UNKNOWN cliff — FPVs in
particular fly anywhere from hover to ~40 m/s, so a tight σ around
the 15 m/s mean would collapse MULTIROTOR off-nominal. Short tracks
are damped toward the uniform prior so sparse-data classifications
stay humble.
"""

import math
from typing import Dict, List, Tuple

# (classification, speed_mps, rcs_m2, altitude_m) — locked spec values.
_THREAT_TYPES: Dict[str, Tuple[str, float, float, float]] = {
    "FPV":     ("MULTIROTOR",  15.0, 0.02,  100.0),
    "SHAHED":  ("LOITERING",   57.0, 0.04,  200.0),
    "QASEF":   ("FIXED_WING",  61.0, 0.05, 2500.0),
    "SAMAD":   ("FIXED_WING",  63.0, 0.10, 5000.0),
    "MOHAJER": ("FIXED_WING",  55.0, 0.06, 3000.0),
    "ABABIL":  ("FIXED_WING",  50.0, 0.03, 1500.0),
}

# Altitude σ is set per-type to reflect real flight-profile variance, not a
# uniform fraction of the mean. This keeps adjacent threat bands overlapping
# so off-spec altitudes don't fall into a UNKNOWN cliff between groups.
_ALT_SIGMA: Dict[str, float] = {
    "FPV":      100.0,
    "SHAHED":   250.0,
    "QASEF":    700.0,
    "SAMAD":   1500.0,
    "MOHAJER":  800.0,
    "ABABIL":   500.0,
}

# Speed σ is per-type for the same reason: FPVs in particular have a wide
# operational envelope (hover ~0 to ~40 m/s on attack runs), so a 15%-of-mean
# σ on a 15 m/s mean would treat any non-cruise speed as zero-likelihood.
_SPEED_SIGMA: Dict[str, float] = {
    "FPV":     10.0,
    "SHAHED":   8.0,
    "QASEF":    8.0,
    "SAMAD":    8.0,
    "MOHAJER":  8.0,
    "ABABIL":   8.0,
}

_CLASSES: Tuple[str, ...] = ("MULTIROTOR", "LOITERING", "FIXED_WING", "UNKNOWN")
_RCS_SIGMA_FRAC = 0.3
# Floor log-likelihood acts as the UNKNOWN baseline: any threat type whose
# joint log-likelihood drops below this loses to UNKNOWN in the softmax.
_UNKNOWN_LOG_LIK = -10.0
_AGE_FULL_S = 5.0


def _log_gauss(x: float, mu: float, sigma: float) -> float:
    return -0.5 * ((x - mu) / sigma) ** 2 - 0.5 * math.log(2.0 * math.pi * sigma * sigma)


def _logsumexp(values: List[float]) -> float:
    m = max(values)
    if math.isinf(m):
        return m
    return m + math.log(sum(math.exp(v - m) for v in values))


def classify(
    rcs_m2: float,
    speed_mps: float,
    altitude_m: float,
    track_age_s: float,
) -> Dict[str, float]:
    """Return posterior over {MULTIROTOR, LOITERING, FIXED_WING, UNKNOWN}."""
    type_log_liks: Dict[str, List[float]] = {
        "MULTIROTOR": [],
        "LOITERING": [],
        "FIXED_WING": [],
    }

    for name, (cls, sp_mu, rcs_mu, alt_mu) in _THREAT_TYPES.items():
        sp_sigma = _SPEED_SIGMA[name]
        rcs_sigma = max(_RCS_SIGMA_FRAC * rcs_mu, 1e-6)
        alt_sigma = _ALT_SIGMA[name]
        ll = (
            _log_gauss(speed_mps, sp_mu, sp_sigma)
            + _log_gauss(rcs_m2, rcs_mu, rcs_sigma)
            + _log_gauss(altitude_m, alt_mu, alt_sigma)
        )
        type_log_liks[cls].append(ll)

    cls_ll: Dict[str, float] = {cls: _logsumexp(lls) for cls, lls in type_log_liks.items()}
    cls_ll["UNKNOWN"] = _UNKNOWN_LOG_LIK

    m = max(cls_ll.values())
    exps = {cls: math.exp(ll - m) for cls, ll in cls_ll.items()}
    total = sum(exps.values())
    posterior = {cls: exps[cls] / total for cls in _CLASSES}

    age = max(track_age_s, 0.0)
    w = min(age / _AGE_FULL_S, 1.0)
    uniform = 1.0 / len(_CLASSES)
    blended = {cls: w * posterior[cls] + (1.0 - w) * uniform for cls in _CLASSES}

    s = sum(blended.values())
    return {cls: blended[cls] / s for cls in _CLASSES}
