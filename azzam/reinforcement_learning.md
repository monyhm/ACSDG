# Reinforcement learning

## Why RL

The rule-based dispatcher (scorer + WTA) clears Easy and Medium scenarios well but degrades on Hard — saturation, decoys, multi-axis attacks, sensor degradation. Hard scenarios reward *temporal* tradeoffs (hold an interceptor for the next salvo vs. shoot now) that the greedy WTA can't express. RL gives us a policy that can defer.

## V1 — PPO baseline

Fixed-size environment in `~/acsdg-backend/rl/environment.py` (not in this folder; baseline reference). 4 interceptors, 6 threats, flat 60-dim observation, Discrete(25) action.

### Curriculum (V1)

Six stages, each ~100k–250k steps, increasing in complexity:
1. Foundation — clean radar, all real, head-on
2. Scarcity — first decoys (20%)
3. Multi-axis — multiple attack vectors
4. Spoofing — decoys with elevated RCS
5. Saturation — n_threats > n_interceptors
6. Degraded — sensor noise per-quadrant

### Results

| Tier   | Rule-based | PPO baseline | Δ       |
|--------|-----------:|-------------:|--------:|
| Easy   |        95% |          98% |  +3.0pp |
| Medium |        77% |          82% |  +5.0pp |
| Hard   |        54% |        63.5% |  +9.5pp |

Hard-tier gain is the headline number — the baseline RL is meaningfully better than rule-based on the scenarios that actually matter operationally. Easy-tier gain is small because the rule-based policy was already near-ceiling.

## V2 — Attention policy (in development)

V1's flat-vector observation hard-codes `n_threats=6, n_interceptors=4`. That can't generalize across configs (saturation, swarm, partial-base outage). V2 reshapes the observation as variable-length sets and lets a self-attention encoder do the matching.

### Variable-config environment (`code/env_v2.py`)

`AdamantEnvV2` accepts arbitrary `(n_threats, n_interceptors)` up to `(max_threats=20, max_interceptors=16)`. Observation is a `gym.spaces.Dict`:

```python
{
    "threats":          Box(shape=(max_threats, 9)),
    "interceptors":     Box(shape=(max_interceptors, 5)),
    "threat_mask":      Box(shape=(max_threats,)),       # 1.0 valid, 0.0 padding
    "interceptor_mask": Box(shape=(max_interceptors,)),
}
```

Action space: `MultiDiscrete([max_threats + 1] * max_interceptors)` — each interceptor picks a threat index or `max_threats` ("hold"). Mask propagation lets the attention head ignore padding rows.

Physics matches the live backend: 100×100 grid, 200 m/unit, 2 Hz tick, threat specs locked.

### 6-component shaped reward (`code/reward_v2.py`)

| Component       | Range           | Activation               | Purpose |
|-----------------|-----------------|--------------------------|---------|
| `survival`      | -200 / 0 / +200 | terminal                 | win/lose terminal signal |
| `coverage`      | 0 — 0.8         | per step                 | shape: keep bases ready across quadrants |
| `depletion`     | -1 / -0.3 / 0   | per step                 | shape: avoid bottoming out the interceptor pool |
| `kill_bonus`    | non-negative    | per kill event           | shape: kill far from asset (≥3 km) |
| `waste_penalty` | -10 × N         | per multi-engagement     | shape: don't double-assign |
| `geometry`      | -1 / 0 / +1     | per kill event           | shape: prefer perpendicular intercepts |

Total = sum of components. The reward function asserts `abs(total - sum(parts)) < 1e-6` on every call.

### Hardened curriculum (7 stages)

Adds a 7th "hardened" stage on top of V1's curriculum: variable `(n_threats, n_interceptors)` per episode, drawn to exercise both scarcity (`n_int < n_thr`) and saturation (`n_thr >> n_int`). The variable-config env supports this directly via the `scenario="mixed"` constructor argument.

### Test coverage

48 tests passing on the v2 stack:
- 15 environment tests (instantiation, reproducibility, padding masks, multi-engagement dedupe, asset hit, decoy)
- 18 reward unit tests (every component + invariant `total == sum(parts)`)
- 5 smoke configs (random policy × 100 episodes per config; checks NaN/inf, episode length, kill rate, component variance)
- 10 pre-existing classifier tests still pass

The smoke test runs `(4,4), (4,6), (4,8), (8,10), (16,12)` and validates the env+reward stack end-to-end.

### Status

The env + reward + tests are landed. **Not yet trained.** The next session is the attention encoder + PPO training run on the 7-stage curriculum, then evaluation against rule-based and V1 PPO at all three difficulty tiers.
