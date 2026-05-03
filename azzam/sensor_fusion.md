# Sensor fusion

Three components run in series for every track update: Kalman → multi-tower fuse → classify. EMA smoothing on the speed estimate damps jitter into the scoring stage.

## Kalman tracker (`code/kalman.py`)

**6-state**: `[x, y, z, vx, vy, vz]`. Constant-velocity motion model — drones are not maneuvering aggressively at the timescales we care about, and over-eager process noise makes the filter chase sensor jitter.

- **Internal tick**: 100 ms (`dt = 0.1`). Higher rate than the dispatcher to absorb between-tower arrival jitter without smearing it into velocity.
- **State transition `F`**: identity + `dt` on the position-velocity coupling rows.
- **Measurement `H`**: position-only (`x, y, z`). Towers don't directly measure velocity; we infer it from successive positions.
- **Initial covariance `P`**: position σ² ≈ 2 (xy), 10 (z, since altitude estimates from the towers are looser); velocity σ² ≈ 0.5.
- **Measurement noise `R`**: 25 across the diagonal — *higher* than feels natural, deliberately. We trust the constant-velocity model more than any single tower reading.
- **Process noise `Q`**: very low velocity component (0.001) to encode "velocity changes slowly". Position component is small but nonzero so the filter can absorb genuine altitude drift.

`predict()` is called once per 100 ms tick across the worker. `update()` is called for each tower reading that arrives during that tick, in arrival order. The `_predicted` flag prevents double-prediction within a tick.

## Multi-tower fusion (`code/fusion_engine.py`)

Tower readings for the same physical track arrive at slightly different times and with different sensor characteristics (radar, EO/IR, RF). Fusion responsibility:

1. **Track association** — match incoming detections to the right Kalman tracker. Identity is held over a short re-acquisition window so brief dropouts don't spawn ghost tracks.
2. **Predict-then-update** — one `predict()` per tick at the start, then `update()` per tower reading. Avoids the classic "predict per measurement" bug that smears velocity.
3. **EMA speed smoothing** — `smoothed = 0.15 * speed + 0.85 * prev`. Alpha is deliberately low (α=0.15) because the scorer is sensitive to speed jumps and we'd rather lag a true speed change by a tick than chase a sensor spike.
4. **Classification locking** — once the posterior gives ≥0.7 weight to one class for ≥3 consecutive ticks, the track's classification locks until the track terminates. Prevents the operator UI from flickering between classes when a track sits near a class boundary.

## Bayesian probabilistic classifier (`code/classifier.py`)

Maps `(rcs_m2, speed_mps, altitude_m, track_age_s)` to a posterior over `{MULTIROTOR, LOITERING, FIXED_WING, UNKNOWN}`.

### Likelihood model

For each of the 6 known threat types, the joint log-likelihood is the sum of three Gaussian log-densities — one each on RCS, speed, and altitude. Each type's σ is calibrated per-feature:

- **RCS σ** = 30% of mean (uniform across types — RCS is well-characterized).
- **Speed σ** is per-type. FPVs in particular have a wide operational envelope (hover ~0 to ~40 m/s on attack runs), so a tight σ around the 15 m/s mean would collapse MULTIROTOR off-nominal. FPV gets σ=10; everything else σ=8.
- **Altitude σ** is per-type for the same reason. FPV σ=100, Shahed σ=250, Qasef σ=700, Samad σ=1500, Mohajer σ=800, Ababil σ=500. Without per-type altitude variance, a Samad cruising 1000 m below spec would fall into the UNKNOWN cliff between FIXED_WING types.

Each known type's log-likelihood is summed via `logsumexp` into the type's parent class. UNKNOWN gets a constant floor log-likelihood of `-10.0` — any class whose joint log-likelihood drops below that floor loses to UNKNOWN in the softmax.

### Track-age damping

For tracks younger than 5 seconds, the posterior is blended toward the uniform prior linearly with age: `blended = (age/5) * posterior + (1 - age/5) * uniform`. A 0.5-second track with one tower hit gets ~10% weight on its highest-likelihood class and ~90% on uniform — which translates downstream into low-confidence dispatching, the right behavior for a fresh track.

## Output to scorer

The fusion engine emits per-track:
- Position `(x, y, z)` in grid units
- Velocity `(vx, vy, vz)` in grid units / second
- Classification posterior (4-element dict)
- Locked classification (or `null` if not yet locked)
- Confidence (max of the posterior)
- Track age, hit count, RCS, smoothed speed

The scorer (`code/scorer.py`) consumes these and emits a 0–100 priority that the WTA stage maximizes over.
