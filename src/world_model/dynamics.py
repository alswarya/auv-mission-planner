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
    raise NotImplementedError


def step(eta: np.ndarray, nu: np.ndarray, thrust: np.ndarray,
         current: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """One integration step of the analytical model. Residual MLP correction
    is applied on top in world_model/residual.py."""
    raise NotImplementedError
