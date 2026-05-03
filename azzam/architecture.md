# System architecture

## Topology

The backend runs as 9 cooperating async workers behind a Redis pub/sub bus, with PostgreSQL for persistent logging and a WebSocket bridge for the React operator dashboard.

```
            ┌───────────────────┐
            │  Jetson / sensors │  (Abdulrahman's /src)
            └─────────┬─────────┘
                      │ HTTP detections
                      ▼
            ┌───────────────────┐
            │ Ingest worker     │
            └─────────┬─────────┘
                      │ Redis: detections.raw
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
  ┌──────────┐  ┌──────────┐  ┌──────────┐
  │ Fusion   │  │ Classifier│ │ Scorer   │
  │ (Kalman) │  │ (Bayes)   │ │          │
  └────┬─────┘  └─────┬─────┘ └────┬─────┘
       │              │             │
       └──── Redis: tracks.{stage} ─┘
                      │
                      ▼
            ┌───────────────────┐
            │ WTA / dispatcher  │
            └─────────┬─────────┘
                      │
              ┌───────┴────────┐
              ▼                ▼
       ┌────────────┐   ┌────────────┐
       │ Interceptor│   │ WebSocket  │
       │ guidance   │   │ bridge →   │
       │ controller │   │ React UI   │
       └────────────┘   └────────────┘
                      │
                      ▼
              ┌────────────┐
              │ PostgreSQL │  (event log, reports)
              └────────────┘
```

## Worker pool (9 total)

1. **Ingest** — pulls Jetson detections, normalizes payloads, publishes to `detections.raw`.
2. **Fusion** — per-track Kalman update across towers, EMA speed smoothing.
3. **Classifier** — Bayesian likelihood-ratio over MULTIROTOR / LOITERING / FIXED_WING / UNKNOWN.
4. **Scorer** — threat priority via proximity, heading, speed, TTI, RCS, confidence.
5. **WTA** — weighted target assignment, picks one threat per available interceptor.
6. **Dispatcher** — issues guidance commands, manages interceptor lifecycle.
7. **Bridge** — WebSocket fan-out for the operator dashboard.
8. **Logger** — persists every stage transition to PostgreSQL.
9. **Reporter** — periodic Excel rollup for after-action review.

## Grid system

- **100 × 100 units**, asset at `(50, 50)`.
- **1 unit = 200 m** → full grid is **20 km × 20 km**.
- **Tick rate**: 2 Hz (`dt = 0.5 s`) for the live sim path; the Kalman tracker runs at 10 Hz internally to absorb inter-tower jitter.

## Interceptor bases

| Base       | Position    |
|------------|-------------|
| INT-NORTH  | (50, 15)    |
| INT-SOUTH  | (50, 85)    |
| INT-EAST   | (85, 50)    |
| INT-WEST   | (15, 50)    |

Bases sit on the asset axes by design — they're omnidirectional defense points. (This shows up as a constraint in the v2 reward's coverage component; see `reinforcement_learning.md`.)

Max interceptor speed: **80 m/s** (MARSS MR spec). Acceleration: 20 m/s².

## Threat specs (locked)

| Type    | Speed (m/s) | RCS (m²) | Altitude (m) | Class       |
|---------|------------:|---------:|-------------:|-------------|
| FPV     |          15 |     0.02 |          100 | MULTIROTOR  |
| Shahed  |          57 |     0.04 |          200 | LOITERING   |
| Qasef   |          61 |     0.05 |         2500 | FIXED_WING  |
| Samad   |          63 |     0.10 |         5000 | FIXED_WING  |
| Mohajer |          55 |     0.06 |         3000 | FIXED_WING  |
| Ababil  |          50 |     0.03 |         1500 | FIXED_WING  |

Speed σ and altitude σ are per-type (real flight-profile variance, not a uniform fraction of mean) so that adjacent threat bands overlap and off-spec readings don't fall into an UNKNOWN cliff. See `code/classifier.py` for the exact σ table.

## HTTP contract with /src

The ingest worker accepts POST `/detections` from the Jetson side. Payload is a list of detections per tower per tick; the contract is bearer-token authenticated and idempotent on `(tower_id, tick, detection_id)`. Field schema lives in the ingest adapter — Abdulrahman owns the producer side.
