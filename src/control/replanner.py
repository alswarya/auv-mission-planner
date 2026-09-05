"""MPC-style closed-loop replanner with trust-region guard.

The LLM is NOT re-queried here. On deviation from the surrogate prediction,
re-run duration optimization over the remaining plan inside the trust region.
Ablation result to reproduce: 100% vs 10% goal-reach on the tight slalom
with vs without the guard (arXiv:2608.19661).
"""


def replan(plan, state, surrogate, trust_region_cfg) -> "plan":
    raise NotImplementedError
