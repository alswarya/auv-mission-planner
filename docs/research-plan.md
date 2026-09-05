# Research plan

## Milestone 1 - World model
- Implement Fossen 6-DOF dynamics for a BlueROV2 Heavy (mass 13.5 kg, 8x T200,
  first-order actuator lag). See Table I of arXiv:2608.19661.
- Collect rollout data in HoloOcean: random + scripted action sequences with
  (state, action, duration) -> next state.
- Train residual MLP on top of the analytical model; target sub-centimeter
  open-loop accuracy over 60 s rollouts (reference paper's AUV result).

## Milestone 2 - LLM planner
- Define action primitives: move(direction, distance), turn(heading),
  hover(duration), survey_leg(pattern params).
- Prompt template: mission text + vehicle constraints (no-strafe is N/A for
  BlueROV2's full actuation, but thruster limits apply) + obstacle list
  + battery model.
- Parse and validate LLM output into typed plan objects.

## Milestone 3 - Optimizer + replanner
- Three-phase gradient-based duration optimization over the surrogate.
- MPC-style closed loop in sim: execute, observe, replan on deviation.
- Trust-region guard; ablate it (reference paper: 100% vs 10% goal-reach on the
  tightest slalom without the guard).

## Milestone 4 - Evaluation + transfer
- Benchmark mission suite (see README eval plan).
- Transfer to Gazebo + ArduSub SITL under current/waves; report sim-to-sim gap.
- Write up results table + rollout videos for the portfolio artifact.
