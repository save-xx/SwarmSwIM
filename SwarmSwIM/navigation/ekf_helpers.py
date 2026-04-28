import numpy as np


def _is_surface_agent(self, agent):
    return agent.name in self.surface_agents

# ==========================================================
# Velocity helpers
# ==========================================================

def _get_body_velocity_measurement(agent, sim):
    body_vel = _get_body_velocity_from_agent(agent, sim)
    return body_vel

def _get_body_velocity_from_agent(agent, sim):
    if hasattr(agent, "emulated_velocities"):
        vel = np.asarray(agent.emulated_velocities, dtype=float).reshape(2)
        return vel.copy()
    return np.zeros(2, dtype=float)

def _body_to_ned_2d(v_body, psi_deg):
    psi = np.deg2rad(psi_deg)
    sinpsi = np.sin(psi)
    cospsi = np.cos(psi)
    R_mat = np.array([[cospsi, sinpsi], [sinpsi, -cospsi]], dtype=float)
    return R_mat @ np.asarray(v_body, dtype=float).reshape(2)

# ==========================================================
# EKF helpers
# ==========================================================

def _constvel_jacobian(x, dt):
    psi = np.deg2rad(x[3])
    u = float(x[4])
    v = float(x[5])

    c = np.cos(psi)
    s = np.sin(psi)
    deg2rad = np.pi / 180.0

    F = np.eye(6)
    F[0, 3] = dt * (-s * u + c * v) * deg2rad
    F[0, 4] = dt * c
    F[0, 5] = dt * s
    F[1, 3] = dt * (c * u + s * v) * deg2rad
    F[1, 4] = dt * s
    F[1, 5] = -dt * c
    return F

def _ekf_update_linear(st, z, h, H, R, angle_idx=None):
    z = np.asarray(z, dtype=float).reshape(-1)
    h = np.asarray(h, dtype=float).reshape(-1)
    H = np.asarray(H, dtype=float)
    R = np.asarray(R, dtype=float)

    nu = z - h
    if angle_idx is not None:
        nu[angle_idx] = _wrap_deg(nu[angle_idx])

    S = _symmetrize(H @ st.P @ H.T + R)

    try:
        S_inv = np.linalg.inv(S)
    except np.linalg.LinAlgError:
        return

    K = st.P @ H.T @ S_inv

    st.x = st.x + K @ nu
    st.x[3] = _wrap_deg(st.x[3])

    I = np.eye(st.P.shape[0])
    st.P = (I - K @ H) @ st.P @ (I - K @ H).T + K @ R @ K.T
    st.P = _symmetrize(st.P)

# ==========================================================
# History helpers
# ==========================================================

def _append_history_snapshot(st, t, x, P, meas_depth, meas_heading, meas_body_vel):
    st.history.append({
        "kind": "snapshot",
        "t": float(t),
        "x": np.asarray(x, dtype=float).copy(),
        "P": np.asarray(P, dtype=float).copy(),
        "meas_depth": float(meas_depth),
        "meas_heading": float(meas_heading),
        "meas_body_vel": np.asarray(meas_body_vel, dtype=float).copy(),
    })
    _trim_history(st)

def _trim_history(st):
    history_length=1000,
    if len(st.history) > history_length:
        st.history[:] = st.history[-history_length:]

def _get_history_index_before_or_equal(st, t_query):
    idx = None
    for k, entry in enumerate(st.history):
        if float(entry.get("t", -np.inf)) <= float(t_query) + 1e-12:
            idx = k
        else:
            break
    return idx

def _get_nearest_snapshot_index_before(st, start_idx):
    for k in range(start_idx, -1, -1):
        if st.history[k].get("kind") == "snapshot":
            return k
    return None

# ==========================================================
# Utilities
# ==========================================================

def _wrap_deg(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0

def _symmetrize(M):
    M = np.asarray(M, dtype=float)
    return 0.5 * (M + M.T)

def _parse_sender_cov(cov_payload):
    try:
        arr = np.asarray(cov_payload, dtype=float)
    except Exception:
        return None

    if arr.ndim == 0:
        val = float(arr)
        if val < 0.0:
            return None
        return np.eye(3) * val

    if arr.shape == (3,):
        if np.any(arr < 0.0):
            return None
        return np.diag(arr)

    if arr.shape == (3, 3):
        arr = 0.5 * (arr + arr.T)
        if np.any(np.linalg.eigvalsh(arr) < -1e-10):
            return None
        return arr

    return None