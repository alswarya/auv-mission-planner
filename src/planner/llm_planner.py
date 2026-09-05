"""LLM mission planner: natural-language mission -> typed action plan.

Called once per mission. The world model guard (src/world_model) validates and
durations every proposed step before execution; the replanner (src/control)
handles deviations without re-querying the LLM.
"""
from dataclasses import dataclass, field


@dataclass
class PlanStep:
    primitive: str          # move | turn | hover | survey_leg
    params: dict = field(default_factory=dict)
    duration_s: float | None = None  # filled in by the world model guard


@dataclass
class MissionContext:
    mission_text: str
    vehicle_constraints: dict
    obstacles: list
    battery_pct: float


SYSTEM_PROMPT = """You are an AUV mission planner. Decompose the mission into a
sequence of primitives: move(direction_deg, distance_m), turn(heading_deg),
hover(duration_s), survey_leg(width_m, height_m, legs).
Respect the vehicle constraints and avoid all listed obstacles.
Output JSON: {"steps": [{"primitive": ..., "params": {...}}, ...]}.
Do not specify durations - the physics model decides how long each step runs.
"""


def plan(ctx: MissionContext, model: str = "gpt-4o-mini") -> list[PlanStep]:
    """Call the LLM once and parse the response into validated PlanSteps."""
    raise NotImplementedError
