# Handoff — current state of the backend / RL stack

## Branch & repo

- **Backend repo**: `~/acsdg-backend` (private, my working copy)
- **Active branch**: `v2-attention`
- **Files in this PR** are exact byte copies from that branch as of the contribution snapshot.

## Recent commit history (backend `v2-attention`)

```
85517cf v2: kill-rate bound applies only under scarcity; surplus-interceptor configs are trivially clearable by design
c782fc3 v2: fix coverage quadrant boundaries and waste variance test for sparse configs
2a8f7de v2: 6-component shaped reward with per-component logging and smoke test
6716a76 v2: dedupe kills and track waste events for clean reward signaling
dbd5de0 v2: variable-config environment with dict observation space and mask propagation
2273672 v2: per-type speed variance for FPV operational envelope
e390291 v2: remove unreachable code in FusionEngine.update()
92a2927 v2: pre-v2 baseline - EMA smoothing, sensor noise tuning, classification locking, fix locked_threat_type init and field-mapping bugs
ebbf713 v2: per-type altitude variance to eliminate classification discontinuity
f33973b v2: probabilistic threat classifier with Bayesian likelihood ratios
```

The first three commits in this list (`dbd5de0` → `85517cf`) are the V2 RL stack. The four before that are sensor-fusion / classifier work that hardened the rule-based path against off-spec readings.

## What's currently passing

```
$ python3 -m pytest tests/ -v
========================= 48 passed, 1 warning in 4.4s =========================
```

Breakdown:
- `tests/test_classifier.py` — 10 (pre-existing)
- `tests/test_environment_v2.py` — 15
- `tests/test_reward_v2.py` — 18
- `tests/test_v2_smoke.py` — 5

The 1 warning is the geometry-component once-per-process warning when a kill event is missing `geometry_score` (a unit test deliberately exercises the absent-field path).

## What's NOT in the PR

- The 9-worker async orchestrator (`main.py`, adapters, bridge) — not yet ready for external review; lives in the backend repo only.
- Trained PPO baseline weights — large binaries, not appropriate for a code review.
- Tensorboard logs / Excel reports — generated artifacts, not source.
- The fixed-size V1 environment (`rl/environment.py`) — kept in the backend as a reference; superseded by V2 for new training.

## Next session — V2 attention training

1. **Attention encoder + policy head.** Self-attention over the threats slot (with `threat_mask`); cross-attention from interceptor queries to threat keys; per-interceptor action logits with autoregressive masking so the policy can't double-assign without explicit motivation.
2. **PPO training run** on the 7-stage hardened curriculum. Estimate ~2M steps; tensorboard checkpoints every 50k.
3. **Evaluation harness** parity: re-run the same Easy/Medium/Hard scenarios used for the V1 baseline so the comparison is apples-to-apples.
4. **Promotion gate**: V2 promotes to live dispatch only if Hard-tier kill rate exceeds the V1 PPO baseline (63.5%) by ≥5pp.

## Open questions for review

- **Coverage quadrant convention** (`code/reward_v2.py:_coverage`). I deviated from the spec's strict `>` to `>=` on one bound per quadrant because the default bases sit on the asset axes — strict comparison made coverage structurally 0. Documented inline. Open to alternative encodings if you'd rather move the bases or make `INTERCEPT` status persist.
- **Geometry score on speed=0 setups.** Returns 0 (which lands in the "bad" band per the threshold rules), so manually-rigged tests with stationary interceptors will see a -1 geometry penalty. Acceptable for tests; flagged in case it surprises someone reading the warnings.
- **Waste-vs-miss distinction.** Currently, in a multi-engagement, all non-credited interceptors are "wasted" regardless of whether their individual hit roll would have succeeded. This penalizes redundant *allocation*, not redundant *firing*. Open to splitting that signal further if reward shaping wants finer control.
