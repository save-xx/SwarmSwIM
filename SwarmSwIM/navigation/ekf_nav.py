import copy
from dataclasses import dataclass, field
import numpy as np

from .base_nav import BaseNavFilter


@dataclass
class NavState:
    """
    EKF state container for one agent.
    State:
        x = [x, y, z, psi, u, v]^T
    """
    x: np.ndarray
    P: np.ndarray
    t: float

    last_local_update: float = 0.0
    last_coop_update: float = 0.0
    quality: float = np.inf

    # shadow model used for prediction with the same simulator dynamics
    model_agent: object = None
    prev_true_pos: np.ndarray | None = None

    # optional bookkeeping
    history: list = field(default_factory=list)


class EKFNavFilter(BaseNavFilter):
    """
    Distributed Cooperative EKF for SwarmSwIM.

    State
    -----
    x = [x, y, z, psi, u, v]^T

    where:
    - x, y, z are NED position
    - psi is heading [deg]
    - u, v are body-frame planar velocities

    Notes
    -----
    - Dead reckoning is velocity-driven:
          x_{k+1}, y_{k+1} are propagated from [u, v, psi]
    - The shadow simulator is still used to propagate the nominal heading,
      depth, and nominal body velocity consistently with the simulator model.
    - The transition Jacobian is computed numerically.
    - Local update uses depth + heading + body velocity.
    - Cooperative ranging update is kept in the file but disabled for now.
    - Velocity measurement noise is kept in the code but commented out.
    """

    def __init__(
        self,
        Q_diag=(0.05, 0.05, 0.02, 1.0, 0.05, 0.05),
        R_depth=0.25,
        R_heading_deg2=9.0,
        R_body_vel_diag=(0.05, 0.05),
        R_range=1.0,
        P0_diag=(4.0, 4.0, 1.0, 25.0, 0.25, 0.25),
        max_packet_age=np.inf,
        age_inflation=0.0,
        min_range=1e-6,
        writeback=True,
        keep_history=False,
        rng_seed=0,
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg2)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))
        self.R_range = float(R_range)
        self.P0_diag = np.asarray(P0_diag, dtype=float)

        self.max_packet_age = float(max_packet_age)
        self.age_inflation = float(age_inflation)
        self.min_range = float(min_range)

        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.rng = np.random.default_rng(rng_seed)

    # ==========================================================
    # Base hooks
    # ==========================================================

    def _init_filter(self, agent):
        vel0 = self._get_initial_body_velocity(agent)

        x0 = np.array([
            float(agent.pos[0]),
            float(agent.pos[1]),
            float(agent.pos[2]),
            float(agent.psi),
            float(vel0[0]),
            float(vel0[1]),
        ], dtype=float)

        P0 = np.diag(self.P0_diag)
        model_agent = copy.deepcopy(agent)

        # remove stochasticity in predictor
        for attr in (
            "e_depth", "e_heave", "e_heading", "e_yawrate",
            "e_position", "e_local_vel", "e_inertial_vel", "e_local_force"
        ):
            if hasattr(model_agent, attr):
                setattr(model_agent, attr, np.zeros(2))

        return NavState(
            x=x0,
            P=P0,
            t=0.0,
            last_local_update=0.0,
            last_coop_update=0.0,
            quality=float(np.trace(P0[:3, :3])),
            model_agent=model_agent,
            prev_true_pos=np.asarray(agent.pos[:2], dtype=float).copy()
        )

    def predict(self, agent, sim):
        st = self.filters[agent.name]
        dt = float(sim.time - st.t)

        if dt <= 0.0:
            return

        F = self._numerical_transition_jacobian(agent, sim, st)
        x_pred, model_next = self._propagate_shadow(agent, sim, st.model_agent, st.x)

        Q = np.diag(self.Q_diag * dt)

        st.x = x_pred
        st.P = self._symmetrize(F @ st.P @ F.T + Q)
        st.t = float(sim.time)
        st.model_agent = model_next
        st.quality = float(np.trace(st.P[:3, :3]))

        if self.keep_history:
            st.history.append(("predict", sim.time, st.x.copy(), st.P.copy()))

    def update_local(self, agent, sim):
        st = self.filters[agent.name]

        # depth update
        z_depth = np.array([float(agent.measured_depth)], dtype=float)
        H_depth = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=float)

        self._ekf_update_linear(
            st,
            z=z_depth,
            h=np.array([st.x[2]], dtype=float),
            H=H_depth,
            R=np.array([[self.R_depth]], dtype=float),
            angle_idx=None
        )

        # heading update
        z_psi = float(agent.measured_heading)
        H_psi = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]], dtype=float)

        self._ekf_update_linear(
            st,
            z=np.array([z_psi], dtype=float),
            h=np.array([st.x[3]], dtype=float),
            H=H_psi,
            R=np.array([[self.R_heading]], dtype=float),
            angle_idx=0
        )

        # body velocity update
        z_vel = self._get_body_velocity_measurement(agent, sim)

        H_vel = np.array([
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        self._ekf_update_linear(
            st,
            z=z_vel,
            h=st.x[4:6].copy(),
            H=H_vel,
            R=self.R_body_vel,
            angle_idx=None
        )

        st.last_local_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))
        st.prev_true_pos = np.asarray(agent.pos[:2], dtype=float).copy()

        if self.keep_history:
            st.history.append(("local", sim.time, st.x.copy(), st.P.copy()))

    def process_cooperative(self, sim, delivered):
        for receiver_name, msg in delivered.items():
            if msg is None or not getattr(msg, "intact", False):
                continue

            receiver = sim.agents[receiver_name]
            sender_name = msg.sender

            if sender_name is None or sender_name == receiver_name:
                continue

            if not hasattr(receiver, "AcousticRange"):
                continue
            if sender_name not in receiver.AcousticRange:
                continue

            meas = receiver.AcousticRange[sender_name]
            payload = msg.payload

            if payload is None:
                continue
            if "pos" not in payload or "cov" not in payload:
                continue

            p_j = np.asarray(payload["pos"], dtype=float).reshape(-1)
            if p_j.size != 3:
                continue

            P_j = self._parse_sender_cov(payload["cov"])
            if P_j is None:
                continue

            t_tx = float(payload.get("tx_time", meas.get("t_tx", sim.time)))
            t_rx = float(meas.get("t_rx", sim.time))
            packet_age = max(0.0, t_rx - t_tx)

            if packet_age > self.max_packet_age:
                continue

            z = float(meas["range"])

            
            '''self._update_cooperative_range(
                 receiver_name=receiver_name,
                 p_j=p_j,
                 P_j=P_j,
                 z=z,
                 packet_age=packet_age,
                 t_now=sim.time
             )'''

        if self.writeback:
            self._writeback_all(sim)

    # ==========================================================
    # Cooperative range update
    # ==========================================================

    def _update_cooperative_range(self, receiver_name, p_j, P_j, z, packet_age, t_now):
        st = self.filters[receiver_name]

        p_i = st.x[:3].copy()
        diff = p_i - p_j
        r_hat = float(np.linalg.norm(diff))

        if r_hat < self.min_range:
            return

        H_i = np.array([[diff[0] / r_hat, diff[1] / r_hat, diff[2] / r_hat, 0.0, 0.0, 0.0]], dtype=float)
        H_j = np.array([[-diff[0] / r_hat, -diff[1] / r_hat, -diff[2] / r_hat]], dtype=float)

        R_eff = float(self.R_range + (H_j @ P_j @ H_j.T)[0, 0] + self.age_inflation * packet_age)
        R_eff = max(R_eff, 1e-12)

        nu = float(z - r_hat)
        S = float((H_i @ st.P @ H_i.T)[0, 0] + R_eff)

        if S <= 0.0:
            return

        K = (st.P @ H_i.T) / S

        st.x = st.x + (K[:, 0] * nu)
        st.x[3] = self._wrap_deg(st.x[3])

        I = np.eye(st.P.shape[0])
        KH = K @ H_i
        Rm = np.array([[R_eff]], dtype=float)
        st.P = (I - KH) @ st.P @ (I - KH).T + K @ Rm @ K.T
        st.P = self._symmetrize(st.P)

        st.last_coop_update = float(t_now)
        st.quality = float(np.trace(st.P[:3, :3]))

        if self.keep_history:
            st.history.append(("coop", t_now, st.x.copy(), st.P.copy()))

    # ==========================================================
    # Transition model
    # ==========================================================

    def _propagate_shadow(self, agent, sim, model_template, x_state):
        """
        Velocity-driven DR with simulator-consistent nominal dynamics.

        The shadow model is used to propagate:
        - heading psi
        - depth z
        - nominal body velocity [u, v]

        Then x,y are propagated explicitly from the EKF velocity state:
            p_{k+1} = p_k + R(psi_k) [u_k, v_k] dt
        """
        m = copy.deepcopy(model_template)
        dt = float(sim.Dt)

        # impose EKF state on shadow model
        m.pos = np.array([x_state[0], x_state[1], x_state[2]], dtype=float)
        m.psi = float(x_state[3])
        m.Dt = dt

        if hasattr(m, "incurrent_velocity"):
            m.incurrent_velocity = np.array([x_state[4], x_state[5]], dtype=float)

        if hasattr(m, "last_step_pos"):
            m.last_step_pos = np.array([x_state[0], x_state[1], x_state[2]], dtype=float)

        # synchronize commands/inputs from real agent
        for attr in (
            "cmd_depth", "cmd_heave", "cmd_heading", "cmd_yawrate",
            "cmd_planar", "cmd_local_vel", "cmd_forces", "other_forces",
            "depth_control", "heading_control", "planar_control"
        ):
            if hasattr(agent, attr):
                setattr(m, attr, copy.deepcopy(getattr(agent, attr)))

        # deterministic feedback for control laws
        m.measured_depth = float(m.pos[2])
        m.measured_heading = float(m.psi)
        m.measured_pos = m.pos[:2].copy()

        # propagate shadow heading/depth and internal dynamic state
        m._update_heading()
        m._update_depth()
        m._update_planar(m.Dt)

        # nominal body velocity from simulator-consistent shadow model
        vel_body_nom = self._get_body_velocity_from_shadow(m, x_state, dt)

        # velocity-driven dead reckoning for position
        vel_ned = self._body_to_ned_2d(vel_body_nom, x_state[3])

        x_next = np.array([
            float(x_state[0] + vel_ned[0] * dt),
            float(x_state[1] + vel_ned[1] * dt),
            float(m.pos[2]),
            self._wrap_deg(float(m.psi)),
            float(vel_body_nom[0]),
            float(vel_body_nom[1]),
        ], dtype=float)

        # keep shadow pose aligned with propagated EKF state
        m.pos[0] = x_next[0]
        m.pos[1] = x_next[1]
        m.pos[2] = x_next[2]
        m.psi = x_next[3]

        if hasattr(m, "last_step_pos"):
            m.last_step_pos = m.pos.copy()

        return x_next, m

    def _numerical_transition_jacobian(self, agent, sim, st):
        """
        Numerical Jacobian of:
            x_{k+1} = f(x_k, u_k)
        using central finite differences.
        """
        x0 = st.x.copy()
        n = x0.size
        F = np.zeros((n, n), dtype=float)

        eps = np.array([1e-3, 1e-3, 1e-3, 1e-2, 1e-4, 1e-4], dtype=float)

        for k in range(n):
            dx = np.zeros(n, dtype=float)
            dx[k] = eps[k]

            x_plus, _ = self._propagate_shadow(agent, sim, st.model_agent, x0 + dx)
            x_minus, _ = self._propagate_shadow(agent, sim, st.model_agent, x0 - dx)

            diff = self._state_difference(x_plus, x_minus)
            F[:, k] = diff / (2.0 * eps[k])

        return F

    @staticmethod
    def _state_difference(xa, xb):
        d = np.asarray(xa, dtype=float) - np.asarray(xb, dtype=float)
        d[3] = (d[3] + 180.0) % 360.0 - 180.0
        return d

    # ==========================================================
    # Velocity helpers
    # ==========================================================

    def _get_initial_body_velocity(self, agent):
        if hasattr(agent, "incurrent_velocity"):
            vel = np.asarray(agent.incurrent_velocity, dtype=float).reshape(2)
            if np.linalg.norm(vel) > 1e-12:
                return vel.copy()
        return np.zeros(2, dtype=float)

    def _get_body_velocity_measurement(self, agent, sim):
        vel_body = self._get_body_velocity_from_agent(agent, sim)

        vel_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.R_body_vel)),
            size=2
        )

        return vel_body + vel_noise
        # return vel_body + vel_noise

    def _get_body_velocity_from_agent(self, agent, sim):
        """
        Best-effort extraction of body-frame velocity from true motion.
        Uses EKF-side previous true position.
        """
        st = self.filters[agent.name]

        if st.prev_true_pos is not None:
            dt = float(sim.Dt)
            if dt > 0.0:
                curr_pos = np.asarray(agent.pos[:2], dtype=float)
                vel_ned = (curr_pos - st.prev_true_pos) / dt
                return self._ned_to_body_2d(vel_ned, agent.psi)

        return np.zeros(2, dtype=float)

    def _get_body_velocity_from_shadow(self, model_agent, x_state, dt):
        """
        Extract nominal body velocity for propagation.

        Priority:
        - for local_forces: use dynamic state incurrent_velocity
        - otherwise: use the EKF state velocity itself, since DR is velocity-driven
        """
        if getattr(model_agent, "planar_control", "") == "local_forces":
            if hasattr(model_agent, "incurrent_velocity"):
                return np.asarray(model_agent.incurrent_velocity, dtype=float).reshape(2).copy()

        return np.asarray(x_state[4:6], dtype=float).copy()

    @staticmethod
    def _ned_to_body_2d(v_ned, psi_deg):
        """
        Convert planar NED velocity [vx, vy] to body velocity [u, v].

        Simulator convention:
            velocity_ned = R_mat @ vel_body
        with
            R_mat = [[cos(psi),  sin(psi)],
                     [sin(psi), -cos(psi)]]

        Therefore:
            vel_body = R_mat.T @ velocity_ned
        """
        psi = np.deg2rad(psi_deg)
        sinpsi = np.sin(psi)
        cospsi = np.cos(psi)

        R_mat = np.array([
            [cospsi,  sinpsi],
            [sinpsi, -cospsi]
        ], dtype=float)

        return R_mat.T @ np.asarray(v_ned, dtype=float).reshape(2)

    @staticmethod
    def _body_to_ned_2d(v_body, psi_deg):
        """
        Convert body velocity [u, v] to planar NED velocity [vx, vy].
        """
        psi = np.deg2rad(psi_deg)
        sinpsi = np.sin(psi)
        cospsi = np.cos(psi)

        R_mat = np.array([
            [cospsi,  sinpsi],
            [sinpsi, -cospsi]
        ], dtype=float)

        return R_mat @ np.asarray(v_body, dtype=float).reshape(2)

    # ==========================================================
    # EKF helpers
    # ==========================================================

    def _ekf_update_linear(self, st, z, h, H, R, angle_idx=None):
        z = np.asarray(z, dtype=float).reshape(-1)
        h = np.asarray(h, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)

        nu = z - h

        if angle_idx is not None:
            nu[angle_idx] = self._wrap_deg(nu[angle_idx])

        S = H @ st.P @ H.T + R
        S = self._symmetrize(S)

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        K = st.P @ H.T @ S_inv

        st.x = st.x + K @ nu
        st.x[3] = self._wrap_deg(st.x[3])

        I = np.eye(st.P.shape[0])
        st.P = (I - K @ H) @ st.P @ (I - K @ H).T + K @ R @ K.T
        st.P = self._symmetrize(st.P)

    # ==========================================================
    # Writeback helpers
    # ==========================================================

    def _writeback_all(self, sim):
        for agent in sim.agents.values():
            st = self.filters[agent.name]

            agent.nav_state = st
            agent.est_pos = st.x[:3].copy()
            agent.est_heading = float(st.x[3])
            agent.est_body_vel = st.x[4:6].copy()
            agent.est_cov = st.P.copy()

            agent.nav_info = {
                "traceP_pos": float(np.trace(st.P[:3, :3])),
                "detP_pos": float(np.linalg.det(st.P[:3, :3])),
                "traceP_vel": float(np.trace(st.P[4:6, 4:6])),
                "last_local_update": float(st.last_local_update),
                "last_coop_update": float(st.last_coop_update),
                "quality": float(st.quality),
                "t": float(st.t),
            }

    # ==========================================================
    # Utilities
    # ==========================================================

    @staticmethod
    def _wrap_deg(angle_deg):
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _symmetrize(M):
        M = np.asarray(M, dtype=float)
        return 0.5 * (M + M.T)

    @staticmethod
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