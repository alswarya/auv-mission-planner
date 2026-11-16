"""Unit tests for src/world_model/dynamics.py - run from the project root:

    python tests/test_dynamics.py

No pytest needed, numpy only.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from world_model.dynamics import rotation_matrix  # noqa: E402


def check(name: str, got: np.ndarray, want: np.ndarray, tol: float = 1e-9):
    if not np.allclose(got, want, atol=tol):
        raise AssertionError(f"{name}: got {got}, want {want}")
    print(f"PASS {name}")


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

    print("\nAll tests passed.")


if __name__ == "__main__":
    main()
