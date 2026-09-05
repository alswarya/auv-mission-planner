"""HoloOcean backend: execute a guarded plan and stream state feedback.

Install: pip install holoocean  (https://robots.et.byu.edu/holoocean/)
Vehicle: HoveringAUV or TorpedoAUV; sensors: DVLSensor, IMUSensor, PoseSensor.
"""


def run_mission(plan, scenario: str = "PierHarbor"):
    """Execute plan in HoloOcean; yield (t, state) until done or abort."""
    raise NotImplementedError
