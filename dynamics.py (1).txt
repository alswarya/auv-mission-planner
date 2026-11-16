"""Analytical Fossen-model dynamics for a BlueROV2-class AUV.

State: eta = [x y z roll pitch yaw] (NED), nu = [u v w p q r] (body).
eta_dot = R(eta) nu
M nu_dot + C(nu) nu + D(nu_r) nu_r + g(eta) = tau,  nu_r = nu - nu_c (current)
tau = B f with first-order actuator lag per thruster.

Reference: arXiv:2608.19661 (Sec. III), Fossen, Handbook of Marine Craft
Hydrodynamics and Motion Control.
"""
import numpy as np


def rotation_matrix(eta: np.ndarray) -> np.ndarray:
    """Rotation matrix R mapping body-frame vectors to the NED world frame.

    eta[3:6] are roll (phi), pitch (theta), yaw (psi) in radians.
    R = Rz(psi) @ Ry(theta) @ Rx(phi) - the standard Fossen body-to-NED
    rotation (z-y-x convention). NED z points DOWN, so positive pitch
    rotates the nose up (toward -z).

    To go the other way (NED -> body), use R.T - rotation matrices are
    orthonormal, so the transpose is the inverse.
    """
    phi, theta, psi = eta[3], eta[4], eta[5]
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    rx = np.array([[1, 0, 0],
                   [0, cphi, -sphi],
                   [0, sphi, cphi]])
    ry = np.array([[cth, 0, sth],
                   [0, 1, 0],
                   [-sth, 0, cth]])
    rz = np.array([[cpsi, -spsi, 0],
                   [spsi, cpsi, 0],
                   [0, 0, 1]])
    return rz @ ry @ rx


def step(eta: np.ndarray, nu: np.ndarray, thrust: np.ndarray,
         current: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """One integration step of the analytical model. Residual MLP correction
    is applied on top in world_model/residual.py."""
    raise NotImplementedError
