# auv-world-model-llm-planner

World-model-grounded LLM mission planning for autonomous underwater vehicles (AUVs).

An LLM turns a natural-language mission ("survey the pier legs, avoid the marked
obstacle field, come home if battery drops below 30%") into a sequence of actions.
A learned physics world model then vets every proposed step - is it feasible, how
long should it run, will it drift into an obstacle - before an MPC-style replanner
executes the mission in a high-fidelity underwater simulator.

The LLM decides *what* to do. The world model decides *how long* - and whether the
plan survives contact with physics.

## Why

LLMs can decompose a mission into plausible steps, but they have no sense of
thruster lag, hydrodynamic coupling, or current drift. Ungrounded LLM plans fail
for physical reasons the model cannot anticipate. Grounding the planner in a fast,
differentiable surrogate of the vehicle dynamics fixes this: in the reference
implementation (Buchholz, Carlucho & Petillot, 2026), both a 6-DOF AUV and a 3-DOF
ASV reached every goal with zero predicted collisions, and transfer to GazeboSim
under ocean current, waves, and thruster dynamics cut goal-distance error by ~93%
(AUV) versus the ungrounded baseline.

## Approach

```
natural-language mission
        |
        v
+-------------------+      propose steps       +----------------------+
|    LLM planner    | -----------------------> |  world model guard   |
| (called once per  |                          | Fossen dynamics +    |
|     mission)      | <----------------------- |  learned residual    |
+-------------------+   feasible steps +       |  (MLP)               |
                        step durations         +----------------------+
                                                        |
                                                        v
+-------------------+      closed-loop         +----------------------+
|  underwater sim   | <----------------------- |  MPC-style replanner |
| HoloOcean /       |      state feedback      |  + trust-region      |
| Stonefish /       |                          |  guard               |
| Gazebo + ArduSub  |                          +----------------------+
+-------------------+
```

Components:

1. **LLM planner** (`src/planner/`) - one call per mission; decomposes the mission
   into parametrized primitives (move, turn, hover, survey-leg) with proposed
   headings/distances.
2. **World model** (`src/world_model/`) - analytical Fossen-model dynamics for a
   BlueROV2-class vehicle plus a learned residual MLP trained on simulator
   rollouts; predicts next state given (state, action, duration).
3. **Trajectory optimizer + replanner** (`src/control/`) - gradient-based duration
   optimization, then MPC-style closed-loop replanning with a trust-region guard.
4. **Sim runners** (`src/sim/`) - mission execution in HoloOcean, with Stonefish /
   Gazebo + ArduSub SITL as alternate backends.

## Sim stack

- **HoloOcean** (BYU FRoStLab): Unreal Engine underwater sim with DVL, IMU, imaging
  sonar, multi-agent support. https://robots.et.byu.edu/holoocean/
  v2.0 (UE5.3, Fossen dynamics, ROS 2 bridge): https://arxiv.org/html/2510.06160v1
- **Stonefish**: open-source marine robotics sim with ML-oriented tooling
  (automated annotation, improved thruster/hydrodynamics models).
  https://github.com/patrykcieslak/stonefish - paper: https://arxiv.org/html/2502.11887v1
- **Gazebo + ArduSub SITL** (BlueROV2): replicates the real autopilot stack for
  sim-to-sim transfer evals. https://github.com/markusbuchholz/gazebosim_bluerov2_ardupilot_sitl

## Evaluation plan

- 5-10 benchmark missions of increasing difficulty (straight-line transit,
  slalom between obstacles, inspection pattern, return-home under current).
- Metrics: goal-distance error, collision rate, plan feasibility rate, energy use.
- Baselines: (a) ungrounded LLM planner (no world model), (b) world model without
  trust-region guard (ablation).
- Transfer: run the same missions in a second simulator under ocean current and
  report the sim-to-sim gap.

## Repo layout

- `missions/` - mission definitions (YAML)
- `configs/` - vehicle params (BlueROV2 Heavy defaults), planner and guard settings
- `src/planner/` - LLM prompt construction and plan parsing
- `src/world_model/` - Fossen dynamics + residual MLP
- `src/control/` - duration optimizer and MPC replanner
- `src/sim/` - simulator backends
- `scripts/` - data collection, residual training, evaluation
- `docs/` - research plan and source ledger
- `results/` - eval tables and rollout logs

## Status

Scaffold. See `docs/research-plan.md` for the build order.

## Key sources

- Buchholz, Carlucho & Petillot, World-Model-Grounded LLM Planning for AUV and ASV
  Navigation Near Offshore Wind Farms: https://arxiv.org/html/2608.19661
- HoloOcean: https://robots.et.byu.edu/holoocean/
- HoloOcean 2.0 preview: https://arxiv.org/html/2510.06160v1
- Stonefish (ML in marine robotics): https://arxiv.org/html/2502.11887v1
