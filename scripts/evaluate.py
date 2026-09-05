"""Run the benchmark mission suite x N seeds; emit results table
(goal-distance error, collision rate, feasibility rate) per condition:
- ungrounded LLM baseline
- world-model grounded
- grounded, no trust-region guard (ablation)
- transfer run in Gazebo + ArduSub SITL under current
"""
