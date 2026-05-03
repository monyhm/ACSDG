# Azzam — Backend, Sensor Fusion, and RL contributions

## What I built

The decision-side stack for the ADAMANT C-UAS pipeline: a 9-worker async backend that ingests Jetson detections, fuses them across towers with a 6-state Kalman filter, classifies threats via Bayesian likelihood ratios, scores them, and dispatches interceptors via Weighted Target Assignment (WTA). On the learning side, a PPO baseline trained on a fixed-size environment, plus a v2 attention-policy stack now in development with a variable-config environment and a 6-component shaped reward.

## Folder structure

```
azzam/
├── README.md                  — this file
├── architecture.md            — system architecture + grid + threat specs
├── sensor_fusion.md           — Kalman + multi-tower fusion + classifier
├── reinforcement_learning.md  — PPO baseline + v2 attention work
├── handoff.md                 — current branch state + next session
├── code/
│   ├── classifier.py          — Bayesian probabilistic classifier
│   ├── fusion_engine.py       — multi-tower fusion + EMA smoothing
│   ├── kalman.py              — 6-state Kalman tracker
│   ├── scorer.py              — threat priority scoring
│   ├── wta.py                 — weighted target assignment
│   ├── env_v2.py              — variable-config RL environment
│   └── reward_v2.py           — 6-component shaped reward
└── diagrams/                  — Figma exports (added manually)
```

## Key results

| Tier        | Rule-based | RL (PPO baseline) | Δ      |
|-------------|-----------:|------------------:|-------:|
| Easy        |       95%  |              98%  |  +3.0pp |
| Medium      |       77%  |              82%  |  +5.0pp |
| Hard        |     54%  |            63.5%  |  +9.5pp |

V2 attention policy (variable-config env + 6-component reward + 7-stage hardened curriculum) is in development. 48 unit + smoke tests passing on the v2 stack.

## How this connects to /src

Abdulrahman's `/src` (Jetson + perception) emits detections over an HTTP message contract. My backend's ingest worker subscribes to that endpoint, normalizes detection payloads into per-tower track updates, and feeds them into `fusion_engine.py`. The contract is bearer-token authenticated; payload schema is documented inline in the ingest adapter. From there, fused tracks flow Redis → fusion → classifier → scorer → WTA → interceptor dispatch, with PostgreSQL logging at every stage for after-action review.

## Status

**Shipped:**
- Async ingest, fusion, classification, scoring, dispatch (rule-based path)
- PPO baseline trained, evaluated, and reported (Excel out)
- V2 environment + reward (this branch contributes the building blocks)

**Next:**
- Train the v2 attention policy on the 7-stage curriculum
- Re-evaluate against rule-based and PPO baseline at all three difficulty tiers
- Promote v2 to live dispatch path once Hard-tier kill rate exceeds the PPO baseline by ≥5pp
