"""Analytical Fossen-model dynamics for a BlueROV2-class AUV.

State: eta = [x y z roll pitch yaw] (NED), nu = [u v w p q r] (body).
eta_dot = J(eta) nu  (J = [R 0; 0 T], the Euler-angle rate transform)
M nu_dot + C(nu) nu + D(nu_r) nu_r + g(eta) = tau,  nu_r = nu - nu_c (current)
tau = B f with first-order actuator lag per thruster (see update_thrust).

Parameters: BlueROV2 Heavy (configs/bluerov2_heavy.yaml), standard Fossen
values. Per issue #1 the T200 thrust limits are placeholders pending measured
thrust-curve data.

Reference: arXiv:2608.19661 (Sec. III), Fossen, Handbook of Marine Craft
Hydrodynamics and Motion Control.
"""
import numpy as np

# --- BlueROV2 Heavy parameters (Fossen notation) ---
MASS = 13.5                     # kg
GRAVITY = 9.81
WEIGHT = MASS * GRAVITY         # N
BUOYANCY = WEIGHT               # neutrally buoyant, CG/CB coincident -> g(eta) = 0
INERTIA = np.array([0.26, 0.23, 0.37])        # Ix Iy Iz, kg m^2
ADDED_MASS = np.array([5.5, 12.7, 14.6, 0.12, 0.12, 0.12])   # -X_udot ...
DAMP_LIN = np.array([4.03, 6.22, 5.18, 0.07, 0.07, 0.07])    # -X_u ...
DAMP_QUAD = np.array([18.18, 21.66, 36.99, 1.55, 1.55, 1.55])  # -X_u|u| ...

# Total diagonal mass matrix M = M_RB + M_A (rigid body + added mass)
M_DIAG = np.concatenate([[MASS] * 3, INERTIA]) + ADDED_MASS

# --- Thrusters: 8x T200, BlueROV2 Heavy frame layout ---
N_THRUSTERS = 8
F_MAX = 40.0    # N per thruster, placeholder forward limit (issue #1)
F_MIN = -32.0   # N per thruster, placeholder reverse limit
T_LAG = 0.15    # s, first-order actuator lag time constant

# Allocation matrix B (6x8): tau = B f. Thrusters 1-4 horizontal at 45 deg
# (moment arms lx, ly), thrusters 5-8 vertical; positive vertical thrust
# pushes UP (-z body, z is down). Arms are approximate frame geometry.
_c45 = np.cos(np.pi / 4)
_LX, _LY = 0.16, 0.11
_YAW = (_LX - _LY) * _c45
ALLOCATION = np.array([
    [ _c45,  _c45,  _c45,  _c45,  0,  0,  0,  0],  # X (surge)
    [ _c45, -_c45,  _c45, -_c45,  0,  0,  0,  0],  # Y (sway)
    [    0,     0,     0,     0, -1, -1, -1, -1],  # Z (heave, up positive)
    [    0,     0,     0,     0, -_LY,  _LY,  _LY, -_LY],  # K (roll)
    [    0,     0,     0,     0,  _LX,  _LX, -_LX, -_LX],  # M (pitch)
    [ _YAW, -_YAW, -_YAW,  _YAW, 0, 0, 0, 0],  # N (yaw)
])


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


def _skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix S such that S(a) b = a x b."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def transformation_matrix(eta: np.ndarray) -> np.ndarray:
    """J(eta) mapping body velocities nu to NED Euler rates: eta_dot = J nu.

    Top-left is R(eta); bottom-right T maps (p q r) to (roll pitch yaw) rates.
    Singular at pitch = +/-90 deg, far from normal AUV operation.
    """
    phi, theta = eta[3], eta[4]
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, tth = np.cos(theta), np.tan(theta)
    t_ang = np.array([[1, sphi * tth, cphi * tth],
                      [0, cphi, -sphi],
                      [0, sphi / cth, cphi / cth]])
    j = np.zeros((6, 6))
    j[0:3, 0:3] = rotation_matrix(eta)
    j[3:6, 3:6] = t_ang
    return j


def _coriolis(nu: np.ndarray) -> np.ndarray:
    """C(nu) for the diagonal M (rigid body + added mass), Fossen form."""
    s1 = _skew(M_DIAG[0:3] * nu[0:3])
    s2 = _skew(M_DIAG[3:6] * nu[3:6])
    c = np.zeros((6, 6))
    c[0:3, 3:6] = -s1
    c[3:6, 0:3] = -s1
    c[3:6, 3:6] = -s2
    return c


def _damping(nu_r: np.ndarray) -> np.ndarray:
    """D(nu_r) nu_r: linear + quadratic diagonal damping on relative velocity."""
    return DAMP_LIN * nu_r + DAMP_QUAD * nu_r * np.abs(nu_r)


def _restoring(eta: np.ndarray) -> np.ndarray:
    """g(eta): weight/buoyancy in body frame. Zero for our neutral, coincident
    CG/CB setup - kept general so a ballast change is a one-line edit."""
    phi, theta = eta[3], eta[4]
    w_minus_b = WEIGHT - BUOYANCY
    return np.array([w_minus_b * np.sin(theta),
                     -w_minus_b * np.cos(theta) * np.sin(phi),
                     -w_minus_b * np.cos(theta) * np.cos(phi),
                     0, 0, 0])


def update_thrust(thrust: np.ndarray, command: np.ndarray, dt: float) -> np.ndarray:
    """Advance the per-thruster force state toward the pilot's command.

    First-order actuator lag per thruster (time constant T_LAG), with
    saturation at the T200 forward/reverse limits. Call this once per step
    and pass the result to step() as `thrust`.
    """
    command = np.clip(np.asarray(command, dtype=float), F_MIN, F_MAX)
    thrust = np.asarray(thrust, dtype=float) + dt / T_LAG * (command - thrust)
    return np.clip(thrust, F_MIN, F_MAX)


def step(eta: np.ndarray, nu: np.ndarray, thrust: np.ndarray,
         current: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """One integration step of the analytical model. Residual MLP correction
    is applied on top in world_model/residual.py.

    eta, nu: current state. thrust: CURRENT per-thruster force (8,) - advance
    it with update_thrust() each step. current: fluid velocity nu_c (6,) in
    the body frame. dt: timestep in seconds.

    Semi-implicit Euler: nu first, then eta from the new nu. Stable at
    dt <= 0.05 s for this model; the rollout scripts use 0.02 s.
    """
    eta = np.asarray(eta, dtype=float)
    nu = np.asarray(nu, dtype=float)
    tau = ALLOCATION @ np.clip(np.asarray(thrust, dtype=float), F_MIN, F_MAX)
    nu_r = nu - np.asarray(current, dtype=float)

    nu_dot = (tau - _coriolis(nu) @ nu - _damping(nu_r) - _restoring(eta)) / M_DIAG
    nu_new = nu + dt * nu_dot
    eta_new = eta + dt * (transformation_matrix(eta) @ nu_new)
    return eta_new, nu_new
