# Source ledger

| Source | URL | Used for |
| --- | --- | --- |
| World-Model-Grounded LLM Planning for AUV/ASV (Buchholz, Carlucho, Petillot 2026) | https://arxiv.org/html/2608.19661 | Core architecture: LLM planner + Fossen+residual world model + 3-phase optimizer + MPC replanner with trust-region guard; benchmark numbers |
| HoloOcean simulator (BYU FRoStLab) | https://robots.et.byu.edu/holoocean/ | Primary sim backend |
| HoloOcean 2.0 preview | https://arxiv.org/html/2510.06160v1 | UE5.3 migration, Fossen dynamics, ROS 2 bridge |
| Stonefish: Supporting ML Research in Marine Robotics | https://arxiv.org/html/2502.11887v1 | Alternate sim backend; annotation tooling |
| Stonefish repo | https://github.com/patrykcieslak/stonefish | Code |
| BlueROV2 Gazebo + ArduSub SITL | https://github.com/markusbuchholz/gazebosim_bluerov2_ardupilot_sitl | Transfer eval backend replicating real autopilot stack |
| UnderwaterVLA (Sci Reports 2026) | https://www.nature.com/articles/s41598-026-63051-8 | Related work: dual-brain VLA for underwater navigation |
| A Cognitive Architecture for Shared Autonomy in AUV Ops | https://arxiv.org/abs/2608.29347 | Related work: ontology + LLM copilot |
| LLM+RL shared autonomy, hull inspection | https://arxiv.org/html/2509.05042 | Related work |
