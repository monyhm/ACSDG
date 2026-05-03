# ACSDG — System Overview (Plain English)

> **The problem:** Drone swarms are cheap. Defending against them is hard. Different threats need different weapons — but humans can't decide fast enough.
>
> **What we built:** An AI that watches the sky, decides which weapon should hit which drone, and orchestrates the whole defense in real time. We simulate it end-to-end in a video-game-quality 3D world.

---

## 1. The 60-second pitch

Imagine a hostile drone swarm flying toward a base. Some are small and slow, some are fast and Iranian-supplied (Shahed-class), some swarm in formation. Each type needs a different counter:

- **Cheap ramming drones** to take out cheap targets
- **High-speed jets** with frag warheads for fast threats
- **Net-capture drones** that catch and recover (so the threat can be analyzed, not just destroyed)
- **Gun turrets** for terminal close-in defense

A defender with all four types is well-equipped — but choosing **which weapon to send at which threat in real time** is a decision a human can't make fast enough. There are dozens of factors: range, speed, kill probability, cost-per-shot, whether the weapon is reloading.

**This is what AI does well.** Our system shows — in a fully working 3D simulation — how an AI commander can:

1. **See** every threat (sensor fusion at 10 Hz)
2. **Decide** the optimal weapon-to-target assignment (math: the Hungarian algorithm + cost matrix)
3. **Dispatch** the right weapon (C++ controllers running 20 Hz flight code)
4. **Adapt** when shots miss or weapons reload (multi-shot cooldown, automatic reassignment)

We're not building drones. We're building the **AI brain that commands them**.

---

## 2. What the demo looks like

```
                    ╔═══════════════════════════════╗
                    ║       INCOMING SWARM          ║
                    ║   4 drones flying at base     ║
                    ╚═══════════════════════════════╝
                                  │
                                  ▼
            ┌──────────────────────────────────────────────┐
            │             AI sees everything                │
            │   "What is each threat? Where? How fast?"     │
            └──────────────────────────────────────────────┘
                                  │
                                  ▼
            ┌──────────────────────────────────────────────┐
            │           AI decides the matchups             │
            │      Hungarian algorithm: best assignment     │
            └──────────────────────────────────────────────┘
                                  │
                                  ▼
            ┌──────────────────────────────────────────────┐
            │         AI dispatches weapons in parallel     │
            │     Each weapon flies its own 20 Hz pursuit   │
            └──────────────────────────────────────────────┘
                                  │
                                  ▼
            ╔═════════════════════════════════════════════╗
            ║              TYPICAL DEMO RESULT              ║
            ║                                               ║
            ║   ✓ Coyote jet     [FRAG-FUZE p50]  1.91 m   ║
            ║   ✓ Anvil drone    NEUTRALISED      7.96 m   ║
            ║   ✓ DroneHunter    [NET-CAPTURE]   14.80 m   ║  ← caught alive
            ║   ✓ Anvil drone    NEUTRALISED      7.98 m   ║
            ║                                               ║
            ║          4 of 4 threats stopped               ║
            ║          0 reached the base                   ║
            ╚═════════════════════════════════════════════╝
```

Each weapon is a real-world model:

| Weapon (in our sim) | Real-world reference | Mass | Speed | Method | Cost |
|---|---|---|---|---|---|
| **Anduril Anvil** | [Anvil quadcopter](https://www.anduril.com/hardware/anvil/) | 5.4 kg | 15 m/s | Kinetic ramming | Cheap |
| **Raytheon Coyote Block 2** | [Coyote](https://www.rtx.com/raytheon/what-we-do/integrated-air-and-missile-defense/coyote) | 7 kg | 160 m/s | Frag-fuze warhead | Mid |
| **Fortem DroneHunter F700** | [F700](https://fortemtech.com/products/dronehunter-f700/) | 18 kg | 31 m/s | Net capture (target survives, can be analyzed) | High but reusable |
| Skyranger 30 (Phase 4) | [Rheinmetall Skyranger 30](https://en.wikipedia.org/wiki/Skyranger_30) | (gun turret) | 1075 m/s muzzle | 30mm AHEAD airburst | Very high |

Together they form a **layered defense** — close-in, mid-range, long-range — and the AI decides who shoots what.

---

## 3. Why this matters (the investor-relevant part)

### 3.1 — The problem is real and growing

- Counter-drone is now a **$3B-and-growing market**. Lockheed Martin invested $25M in Fortem in April 2026.
- The Ukraine war pushed counter-drone tech to the front of every NATO modernization plan.
- Every major prime (Raytheon, Rheinmetall, Anduril, Fortem, Northrop) sells **a weapon** for the swarm problem.
- **Nobody sells the AI that orchestrates them all.** That's an open market.

### 3.2 — Why heterogeneous fleets need AI

A single weapon type has obvious failure modes:
- Anvil-only: cheap but can't catch a fast Shahed
- Coyote-only: fast but burns one shot per kill ($60K each)
- Net-only: humane and recoverable but slow, 3-minute reload

A heterogeneous fleet has **none** of these failure modes — but the optimal "which weapon at which target" decision is a real-time mathematical problem. Our cost matrix + Hungarian algorithm solves it in under a millisecond.

### 3.3 — The simulation is the product, today

We can:
- **Test new weapon combinations** in 5 minutes by editing one Python tuple
- **Demonstrate AI decision-making** without firing any real munitions
- **Sell the brain** to manufacturers who own the bodies
- **Train operators** in a fully-realistic 3D environment before they touch hardware

The ~85 commits of work in this repo represent a **working prototype** of the AI command layer. Not a slide deck — a system you can run end-to-end and watch make real decisions.

---

## 4. What's working today (the proof)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    LIVE END-TO-END DEMO                             │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │  Gazebo physics engine — 4 enemy drones flying inbound  │       │
│   │  ────────────────────────────────────────────────────── │       │
│   │       N                                                 │       │
│   │       │                                                 │       │
│   │   [E1]│            Anvil ●─ ● Coyote ●─ E1              │       │
│   │   E4 ─┼─ E2          NW         NE                      │       │
│   │       │                                                 │       │
│   │   [E3]│            Anvil ●  DroneHunter ●               │       │
│   │       │              SW         SE                      │       │
│   │       │                                                 │       │
│   └────────────────────────│────────────────────────────────┘       │
│                            ▼                                        │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │              AI Command (10 times per second)           │       │
│   │  ┌──────────────────────────────────────────────────┐   │       │
│   │  │  Cost matrix:  weapons × targets                 │   │       │
│   │  │                                                   │   │       │
│   │  │            E1     E2     E3     E4               │   │       │
│   │  │  Coyote  3.1s    7.2s   12s    8.9s              │   │       │
│   │  │  Anvil-N 11s     6.7s   18s    14s               │   │       │
│   │  │  DH-SE   5.6s    11s    8.4s   18s               │   │       │
│   │  │  Anvil-S 14s     11s    9.0s   6.7s              │   │       │
│   │  │                                                   │   │       │
│   │  │     ↓  Hungarian algorithm minimizes total ToI   │   │       │
│   │  │                                                   │   │       │
│   │  │     Coyote → E1   DroneHunter → E3                │   │       │
│   │  │     Anvil-N → E2  Anvil-S → E4                    │   │       │
│   │  └──────────────────────────────────────────────────┘   │       │
│   └─────────────────────────────────────────────────────────┘       │
│                            │                                        │
│                            ▼                                        │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │       Each weapon flies pursuit, kills its target       │       │
│   │       4 of 4 stopped. 0 reached base. 0 friendly fire.  │       │
│   └─────────────────────────────────────────────────────────┘       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**This is not a mockup.** Every kill range above (1.91 m, 7.96 m, 14.80 m, 7.98 m) is a real number from a real run, written by a real C++ flight controller responding to a real engagement order from a real AI dispatcher. The whole thing runs on a $1500 Windows laptop.

---

## 5. The AI's decision-making (in plain English)

When 4 drones come in, the AI has 4 weapons and 4 targets — that's 24 possible weapon-target pairings. It has to pick the best 4 in under 100 milliseconds.

**Step 1.** For every weapon-target pair, compute "**how long until impact?**" That's the **time-to-intercept**. Fast weapons against close targets = short ToI. Slow weapons against fleeing targets = long ToI (or impossible).

**Step 2.** Rule out impossible matchups. A 100m booster-clear distance for a frag jet means it can't engage anything closer than 100m. A drone that's flown out of a weapon's max range is invalid. These cells get marked "infinity" in the cost matrix.

**Step 3.** **Hungarian algorithm.** This is a classical operations-research method (1955) that solves the **assignment problem** optimally: given a cost matrix, find the assignment that minimizes total cost. It runs in under a millisecond on a 4×4 matrix. The output is an exact list of "weapon i should engage target j" pairs.

**Step 4.** Dispatch the orders. Each weapon's flight controller takes over, flies its own pursuit at 20 Hz, and reports back when it kills (or misses).

**Step 5.** When a weapon **misses or finishes a kill**, the AI re-runs the whole loop. If the DroneHunter's net just captured a drone, it can't engage anything for 180 seconds (reload). The AI sees that and routes future threats to the Anvils or Coyote — automatically. **No human in the loop.**

---

## 6. The hidden layers (what makes this AI, not just code)

The simulation has 5 layers that work together:

```
  ┌────────────────────────────────────────────────────────────┐
  │  Layer 5  ║  WEB DASHBOARD                                 │
  │           ║  Operator sees everything in real time         │
  ├───────────╫────────────────────────────────────────────────┤
  │  Layer 4  ║  AI / DECISION                                 │
  │           ║  • C2 engine: cost matrix + Hungarian          │
  │           ║  • Threat predictor (where will it be in 3s?)  │
  │           ║  • Swarm classifier (Bayesian)                 │
  │           ║  • Learning module (RL — Phase 5)              │
  ├───────────╫────────────────────────────────────────────────┤
  │  Layer 3  ║  SENSOR FUSION                                 │
  │           ║  • Radar + RF data combined into clean tracks  │
  │           ║  • Nearest-neighbor association, drop logic    │
  ├───────────╫────────────────────────────────────────────────┤
  │  Layer 2  ║  PER-WEAPON FLIGHT CONTROL                     │
  │           ║  • Each weapon has its own 20 Hz controller    │
  │           ║  • Different physics for jet vs quad vs net    │
  ├───────────╫────────────────────────────────────────────────┤
  │  Layer 1  ║  PHYSICS SIMULATION                            │
  │           ║  • Gazebo Harmonic — full 3D physics           │
  │           ║  • Real flight dynamics, real collision        │
  └───────────────────────────────────────────────────────────┘
```

**Each layer is decoupled.** The AI doesn't know it's running in Gazebo — it could be wired to real radar tomorrow. The flight controllers don't know they're working with a Hungarian-algorithm dispatcher — they could be commanded by a human joystick. This **separation of concerns** is what lets us iterate fast: we built the AI brain, the sensor pipeline, and the flight controllers as independent modules and connected them via standard ROS 2 messaging.

---

## 7. What's done, what's coming

| Phase | Status | Investor takeaway |
|---|---|---|
| **1: Foundation** | ✅ Complete | Modular AI brain, sensor fusion, first weapon class (Anvil), realistic Gazebo physics. **Working end-to-end demo.** |
| **2: Long-range frag jet** | ✅ Complete | Coyote Block 2 added — speed advantage in cost matrix is observable. **Two weapon classes, AI decides between them.** |
| **3: Net-capture interceptor** | ✅ Complete | DroneHunter F700 + multi-shot reload semantics. **Three weapon classes; AI handles asynchronous availability.** |
| **4: Gun turret + classifier** | ⏳ Next | Skyranger 30 (close-in defense) + real Bayesian threat classification (small quad? Shahed-class? group-1?). **Four weapon classes; AI classifies and decides simultaneously.** |
| **5: Showcase polish** | ⏳ Future | Full expected-utility cost function (Pkill × ToI × $cost) + reinforcement learning. **AI optimizes for cost-effectiveness, not just speed.** |

After Phase 5, the system will demonstrate the **complete value chain**: spot a threat, classify what it is, predict where it will be, decide which weapon gives the highest expected utility (kill probability × value of asset defended ÷ weapon cost), dispatch, observe outcome, learn.

That's the product an investor backs.

---

## 8. Why now

- **Counter-drone budgets are exploding.** Ukraine validated the threat. NATO countries are buying.
- **AI-as-the-brain is unclaimed.** Every defense prime sells a weapon. Nobody owns the orchestration layer.
- **Simulation-first is cheap.** We don't need a $50M test range to prove the AI works. We need ~$2K of compute and 100 lines of Python per weapon class.
- **The graduation-project context is asymmetric leverage.** One developer, ~6 weeks, working prototype that runs end-to-end with three real-world-modeled weapon classes. Imagine what a small team could do in 6 months.

---

## 9. The deck-friendly summary

| | |
|---|---|
| **What** | AI-driven counter-drone command system, simulation-first |
| **Why** | Drone swarms are the threat; orchestrating heterogeneous defense is the unsolved problem |
| **How** | Cost-matrix + Hungarian algorithm dispatch, per-weapon flight controllers, sensor fusion, full Gazebo physics |
| **Status** | Working end-to-end demo with 3 weapon classes, 4 simultaneous engagements, 100% kill rate in current scenarios |
| **Next** | Add 4th weapon (gun turret) + real threat classification (Phase 4); add RL learning + cost optimization (Phase 5) |
| **Stack** | ROS 2 Humble, Gazebo Harmonic, Python 3.10, C++17, ~85 commits, 91 automated tests |
| **Repo** | github.com/monyhm/ACSDG (open source) |

---

## 10. The one-line version

> **We built the AI that decides which counter-drone weapon should kill which target — and it works end-to-end in simulation today, with three real-world weapon classes and 100% kill rate against a 4-drone swarm.**
