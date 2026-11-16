"""Unit tests for src/world_model/dynamics.py - run from the project root:

    python tests/test_dynamics.py

No pytest needed, numpy only.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from world_model.dynamics import (F_MAX, N_THRUSTERS, T_LAG,  # noqa: E402
                                  rotation_matrix, step, update_thrust)

DT = 0.02  # s, rollout timestep


def check(name: str, got: np.ndarray, want: np.ndarray, tol: float = 1e-9):
    if not np.allclose(got, want, atol=tol):
        raise AssertionError(f"{name}: got {got}, want {want}")
    print(f"PASS {name}")


def simulate(eta, nu, command, seconds):
    """Roll the model forward with a fixed thruster command."""
    thrust = np.zeros(N_THRUSTERS)
    for _ in range(int(seconds / DT)):
        thrust = update_thrust(thrust, command, DT)
        eta, nu = step(eta, nu, thrust, np.zeros(6), DT)
        assert np.all(np.isfinite(eta)) and np.all(np.isfinite(nu))
    return eta, nu


def main():
    # 1. Zero angles = identity (body frame aligned with NED)
    eta = np.zeros(6)
    check("identity at zero angles", rotation_matrix(eta), np.eye(3))

    # 2. Yaw 90 deg: body x-axis points along NED y (heading east)
    eta = np.array([0, 0, 0, 0, 0, np.pi / 2])
    check("yaw 90: body x -> NED y", rotation_matrix(eta) @ np.array([1, 0, 0]),
          np.array([0, 1, 0]))

    # 3. Pitch +90 deg: body x-axis points to NED -z (nose up, z is down)
    eta = np.array([0, 0, 0, 0, np.pi / 2, 0])
    check("pitch 90: body x -> NED -z", rotation_matrix(eta) @ np.array([1, 0, 0]),
          np.array([0, 0, -1]))

    # 4. Roll 90 deg: body y-axis points along NED +z
    eta = np.array([0, 0, 0, np.pi / 2, 0, 0])
    check("roll 90: body y -> NED z", rotation_matrix(eta) @ np.array([0, 1, 0]),
          np.array([0, 0, 1]))

    # 5. Round trip: world -> body -> world returns the original vector,
    #    at an awkward combined attitude
    eta = np.array([0, 0, 0, 0.4, -0.2, 1.1])
    r = rotation_matrix(eta)
    v = np.array([1.7, -0.3, 2.2])
    check("round trip NED->body->NED", r.T @ (r @ v), v)

    # 6. Proper rotation: orthonormal with determinant +1 (not a reflection)
    check("orthonormal", r.T @ r, np.eye(3))
    assert abs(np.linalg.det(r) - 1.0) < 1e-9
    print("PASS determinant = +1")

    # 7. Zero-thrust hover: neutrally buoyant vehicle at an arbitrary pose,
    #    no thrust, 60 s - nothing may move. Guards g(eta), the integrator
    #    and the allocation matrix against phantom forces.
    eta0 = np.array([1.0, 2.0, 5.0, 0.1, -0.05, 0.7])
    eta1, nu1 = simulate(eta0, np.zeros(6), np.zeros(N_THRUSTERS), 60.0)
    check("zero-thrust hover: pose unchanged (60 s)", eta1, eta0)
    check("zero-thrust hover: stays at rest (60 s)", nu1, np.zeros(6))

    # 8. Damping: released at 0.5 m/s surge with no thrust, drag brings the
    #    vehicle to rest without instability, heading untouched.
    eta2, nu2 = simulate(np.zeros(6), np.array([0.5, 0, 0, 0, 0, 0]),
                         np.zeros(N_THRUSTERS), 60.0)
    assert 0 < nu2[0] < 0.02, f"surge should decay to rest, got {nu2[0]}"
    print("PASS damping settles initial velocity")
    check("damping: no sideways/rotational motion", nu2[1:], np.zeros(5))
    assert eta2[0] > 0.5, "vehicle should have coasted forward"
    print("PASS damping: coasted forward")

    # 9. Constant forward thrust on the 4 horizontal thrusters: pure surge,
    #    settling at the analytical terminal speed (drag balances thrust).
    f_fwd = 5.0  # N per thruster
    command = np.array([f_fwd, f_fwd, f_fwd, f_fwd, 0, 0, 0, 0])
    eta3, nu3 = simulate(np.zeros(6), np.zeros(6), command, 60.0)
    thrust_total = 4 * np.cos(np.pi / 4) * f_fwd
    u_term = (-4.03 + np.sqrt(4.03 ** 2 + 4 * 18.18 * thrust_total)) / (2 * 18.18)
    check("forward thrust: terminal surge speed", nu3[0], u_term, tol=0.02)
    check("forward thrust: pure surge", nu3[1:], np.zeros(5), tol=1e-6)
    check("forward thrust: attitude held", eta3[3:], np.zeros(3))
    assert eta3[0] > 30, f"should travel ~{u_term * 60:.0f} m, got {eta3[0]:.1f}"
    print(f"PASS forward thrust: traveled {eta3[0]:.1f} m in 60 s")

    # 10. Actuator lag: from rest, thrust reaches ~63% of the command after
    #     one time constant and converges to it; saturation clips at F_MAX.
    th = np.zeros(N_THRUSTERS)
    for _ in range(int(T_LAG / DT)):
        th = update_thrust(th, np.full(N_THRUSTERS, 10.0), DT)
    check("lag: 63% after one time constant", th[0], 10 * (1 - 1 / np.e), tol=0.2)
    for _ in range(int(5 / DT)):
        th = update_thrust(th, np.full(N_THRUSTERS, 10.0), DT)
    check("lag: converges to command", th[0], 10.0, tol=1e-3)
    for _ in range(int(5 / DT)):
        th = update_thrust(th, np.full(N_THRUSTERS, 1e3), DT)
    check("lag: saturates at F_MAX", th[0], F_MAX)

    print("\nAll tests passed.")


if __name__ == "__main__":
    main()
