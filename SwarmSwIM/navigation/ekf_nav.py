from dataclasses import dataclass, field

import numpy as np

from .base_nav import BaseNavFilter


@dataclass
class NavState:
    """
    EKF state container for one agent.

    State vector:
        x = [x, y, z, psi, u, v]^T

    where psi is heading in degrees and [u, v] are body-frame velocities.
    """

    x: np.ndarray
    P: np.ndarray
    t: float

    last_local_update: float = 0.0
    last_coop_update: float = 0.0
    quality: float = np.inf

    last_range_time: dict = field(default_factory=dict)
    history: list = field(default_factory=list)


class EKFNavFilter(BaseNavFilter):
    """
    Distributed cooperative EKF for SwarmSwIM.

    State vector:
        x = [x, y, z, psi, u, v]^T

    where psi is heading in degrees and [u, v] are body-frame velocities.
    """

    def __init__(
        self,
        Q_diag=(0.05, 0.05, 0.0001, 0.05, 0.03, 0.03),
        P0_diag=(0.25, 0.25, 1e-4, 9.0, 0.01, 0.01),
        R_depth=1e-6,
        R_heading_deg=4.0,
        R_body_vel_diag=np.array([0.003, 0.003]),
        sigma_rel_speed=0.5,
        R_range=0.25,
        R_corr=1.0,
        R_corr_q_adapt=True,
        R_corr_q_ref=1.0,
        R_corr_q_beta=1.0,
        R_corr_min=0.1,
        R_corr_max=10.0,
        nis_gate=100,#9.5,
        alpha=1.0,
        min_range=1e-2,
        surface_agents=(),
        var_gps_fix=np.array([0.25, 0.25]),
        R_surface_pos_diag=(0.04, 0.04),
        history_length=1000,
        writeback=True,
        keep_history=True,
        rng_seed=50,
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.P0_diag = np.asarray(P0_diag, dtype=float)

        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))

        self.sigma_rel_speed = float(sigma_rel_speed)
        self.R_range = float(R_range)
        self.R_corr = float(R_corr)

        self.R_corr_q_adapt = bool(R_corr_q_adapt)
        self.R_corr_q_ref = float(R_corr_q_ref)
        self.R_corr_q_beta = float(R_corr_q_beta)
        self.R_corr_min = float(R_corr_min)
        self.R_corr_max = float(R_corr_max)

        self.nis_gate = float(nis_gate)

        self.alpha = float(alpha)
        self.min_range = float(min_range)

        self.surface_agents = set(surface_agents)
        self.P_gps_fix = np.diag(np.asarray(var_gps_fix, dtype=float))
        self.R_surface_pos = np.diag(np.asarray(R_surface_pos_diag, dtype=float))

        self.history_length = int(history_length)
        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.rng = np.random.default_rng(rng_seed)
        self.gps_bias = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.P_gps_fix)),
            size=2,
        )

        self.coop_update_log = []
        self.last_range_residual = np.nan

    # ==========================================================
    # Base hooks
    # ==========================================================

    def _init_filter(self, agent):
        vel_body_0 = self._get_body_velocity_measurement(agent, None)

        x0 = np.array(
            [
                float(agent.pos[0]) + self.gps_bias[0],
                float(agent.pos[1]) + self.gps_bias[1],
                float(agent.pos[2]),
                float(agent.psi),
                float(vel_body_0[0]),
                float(vel_body_0[1]),
            ],
            dtype=float,
        )

        P0 = np.diag(self.P0_diag)

        state = NavState(
            x=x0,
            P=P0,
            t=0.0,
            last_local_update=0.0,
            last_coop_update=0.0,
            quality=float(np.trace(P0[:3, :3])),
        )

        if self.keep_history:
            self._append_history_snapshot(
                state=state,
                t=0.0,
                x=state.x,
                P=state.P,
                z_depth=float(agent.measured_depth),
                z_heading=float(agent.measured_heading),
                z_body_vel=np.asarray(vel_body_0, dtype=float),
            )

        return state

    def predict(self, agent, sim):
        state = self.filters[agent.name]
        dt = float(sim.time - state.t)
        if dt <= 0.0:
            return

        F = self._transition_jacobian(state.x, dt)
        Q = np.diag(self.Q_diag * dt)

        state.x = self._propagate_state(state.x, dt)
        state.P = self._symmetrize(F @ state.P @ F.T + Q)
        state.t = float(sim.time)
        state.quality = float(np.trace(state.P[:3, :3]))

        if self.keep_history:
            state.history.append(
                {
                    "kind": "prediction",
                    "t": float(sim.time),
                    "x": state.x.copy(),
                    "P": state.P.copy(),
                }
            )
            self._trim_history(state)

    def update_local(self, agent, sim):
        state = self.filters[agent.name]

        z_depth = np.array([float(agent.measured_depth)], dtype=float)
        z_heading = float(agent.measured_heading)
        z_body_vel = self._get_body_velocity_measurement(agent, sim)

        self._apply_local_measurements(state, z_depth, z_heading, z_body_vel)

        state.last_local_update = float(sim.time)
        state.quality = float(np.trace(state.P[:3, :3]))

        if self.keep_history:
            self._append_history_snapshot(
                state=state,
                t=float(sim.time),
                x=state.x,
                P=state.P,
                z_depth=float(z_depth[0]),
                z_heading=float(z_heading),
                z_body_vel=np.asarray(z_body_vel, dtype=float),
            )

    def update_surface_position(self, agent, sim):
        if not self._is_surface_agent(agent):
            return

        state = self.filters[agent.name]

        gps_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.P_gps_fix)),
            size=2,
        )

        z_xy = np.array(
            [
                float(agent.pos[0]) + gps_noise[0],
                float(agent.pos[1]) + gps_noise[1],
            ],
            dtype=float,
        )

        H_xy = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        R_xy = self.R_surface_pos.copy()

        self._ekf_update_linear(
            state,
            z=z_xy,
            h=state.x[0:2].copy(),
            H=H_xy,
            R=R_xy,
            angle_idx=None,
        )

        state.quality = float(np.trace(state.P[:3, :3]))

        if self.keep_history:
            self._append_history_snapshot(
                state=state,
                t=float(sim.time),
                x=state.x,
                P=state.P,
                z_depth=float(agent.measured_depth),
                z_heading=float(agent.measured_heading),
                z_body_vel=self._get_body_velocity_measurement(agent, sim),
                z_xy=z_xy,
                R_xy=R_xy,
            )

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

            state = self.filters[receiver_name]
            range_meas = receiver.AcousticRange[sender_name]
            payload = msg.payload

            if payload is None:
                continue
            if payload.get("type") != "nav":
                continue
            if "pos" not in payload or "cov" not in payload or "body_vel" not in payload:
                continue

            q_j = float(payload.get("q", np.nan))
            if not np.isfinite(q_j) or q_j < 0.0:
                q_j = np.nan

            t_meas = range_meas.get("t_meas", None)
            if t_meas is None:
                continue
            t_meas = float(t_meas)

            last_t_meas = state.last_range_time.get(sender_name, -np.inf)
            if t_meas <= last_t_meas:
                continue

            p_j_tx = np.asarray(payload["pos"], dtype=float).reshape(-1)
            if p_j_tx.size != 3:
                continue

            vel_j_body = np.asarray(payload["body_vel"], dtype=float).reshape(-1)
            if vel_j_body.size != 2:
                continue

            psi_j_tx = float(payload.get("heading", np.nan))
            if not np.isfinite(psi_j_tx):
                continue

            P_j_tx = self._parse_sender_cov(payload["cov"])
            if P_j_tx is None:
                continue

            t_tx = float(payload.get("tx_time", range_meas.get("t_tx", sim.time)))
            packet_age = max(0.0, float(sim.time) - t_tx)
            z_range = float(range_meas["range"])

            self._update_cooperative_range(
                receiver_name=receiver_name,
                sender_name=sender_name,
                p_j_tx=p_j_tx,
                psi_j_tx=psi_j_tx,
                vel_j_body=vel_j_body,
                P_j_tx=P_j_tx,
                z_range=z_range,
                packet_age=packet_age,
                t_now=float(sim.time),
                t_tx=t_tx,
                t_meas=t_meas,
                q_j=q_j,
            )

            state.last_range_time[sender_name] = t_meas

        if self.writeback:
            self._writeback_all(sim)

    # ==========================================================
    # Cooperative range update
    # ==========================================================

    def _update_cooperative_range(
        self,
        receiver_name,
        sender_name,
        p_j_tx,
        psi_j_tx,
        vel_j_body,
        P_j_tx,
        z_range,
        packet_age,
        t_now,
        t_tx=np.nan,
        t_meas=np.nan,
        q_j=np.nan,
    ):
        state = self.filters[receiver_name]
        P_pre_now = state.P.copy()

        hist_idx = self._get_history_index_before_or_equal(state, t_meas)
        if hist_idx is None:
            return

        hist_entry = state.history[hist_idx]
        if hist_entry.get("kind") != "snapshot":
            hist_idx = self._get_nearest_snapshot_index_before(state, hist_idx)
            if hist_idx is None:
                return
            hist_entry = state.history[hist_idx]

        t_hist = float(hist_entry["t"])
        x_i_hist = hist_entry["x"].copy()
        P_ii_hist = hist_entry["P"].copy()

        dt_i = max(0.0, float(t_meas) - t_hist)
        x_i_meas, P_ii_meas = self._propagate_cov(
            x=x_i_hist,
            P=P_ii_hist,
            dt=dt_i,
        )

        if np.isfinite(t_tx) and np.isfinite(t_meas):
            dt_j = max(0.0, float(t_meas) - float(t_tx))
        else:
            dt_j = 0.0

        x_j_tx = np.array(
            [
                float(p_j_tx[0]),
                float(p_j_tx[1]),
                float(p_j_tx[2]),
                float(psi_j_tx),
                float(vel_j_body[0]),
                float(vel_j_body[1]),
            ],
            dtype=float,
        )

        x_j_meas, P_jj_meas = self._propagate_sender_to_meas(x_j_tx, P_j_tx, dt_j)

        p_i_meas = x_i_meas[:3].copy()
        p_j_meas = x_j_meas[:3].copy()
        delta_p = p_i_meas - p_j_meas

        r_hat = float(np.linalg.norm(delta_p))
        if r_hat < self.min_range:
            return

        H_i = np.array(
            [
                [
                    delta_p[0] / r_hat,
                    delta_p[1] / r_hat,
                    delta_p[2] / r_hat,
                    0.0,
                    0.0,
                    0.0,
                ]
            ],
            dtype=float,
        )

        H_j = np.array(
            [
                [
                    -delta_p[0] / r_hat,
                    -delta_p[1] / r_hat,
                    -delta_p[2] / r_hat,
                ]
            ],
            dtype=float,
        )

        alpha_eff = 0.2 if sender_name in self.surface_agents else self.alpha

        if sender_name in self.surface_agents:
            R_corr_eff = self.R_corr_min
        else:
            R_corr_eff = self._adaptive_R_corr(q_j)

        R_sender = alpha_eff * float((H_j @ P_jj_meas[:3, :3] @ H_j.T)[0, 0])
        R_delay = float((self.sigma_rel_speed * packet_age) ** 2)
        R_eff = float(self.R_range + R_sender + R_delay + R_corr_eff)
        R_eff = max(R_eff, 1e-12)

        nu = float(z_range - r_hat)
        self.last_range_residual = nu

        S = float((H_i @ P_ii_meas @ H_i.T)[0, 0] + R_eff)
        if S <= 0.0:
            return

        nis = float((nu**2) / S)

        if nis > self.nis_gate:
            self.coop_update_log.append(
                {
                    "t": float(t_now),
                    "receiver": receiver_name,
                    "sender": sender_name,
                    "accepted": 0,
                    "range": float(z_range),
                    "r_hat": float(r_hat),
                    "nu": float(nu),
                    "nis": float(nis),
                    "nis_gate": float(self.nis_gate),
                    "S": float(S),
                    "R_eff": float(R_eff),
                    "R_sender": float(R_sender),
                    "R_delay": float(R_delay),
                    "R_corr_eff": float(R_corr_eff),
                    "q_sender": float(q_j) if np.isfinite(q_j) else np.nan,
                    "packet_age": float(packet_age),
                    "t_tx_payload": float(t_tx),
                    "t_meas": float(t_meas),
                    "dt_sender": float(dt_j),
                    "dt_receiver": float(t_now - t_meas),
                    "sender_x_meas": float(p_j_meas[0]),
                    "sender_y_meas": float(p_j_meas[1]),
                    "sender_z_meas": float(p_j_meas[2]),
                    "receiver_x_meas": float(x_i_meas[0]),
                    "receiver_y_meas": float(x_i_meas[1]),
                    "receiver_z_meas": float(x_i_meas[2]),
                    "traceP_pre": float(np.trace(P_pre_now[:3, :3])),
                    "traceP_receiver_meas": float(np.trace(P_ii_meas[:3, :3])),
                    "traceP_sender_meas": float(np.trace(P_jj_meas[:3, :3])),
                }
            )

            print("rejected - nis gate")
            print("receiver", receiver_name)
            print("sender", sender_name)
            print("nis", nis)
            print("nu", nu)
            print("S", S)
            print("z_range", z_range, "r_hat", r_hat)
            print("t_now", t_now, "t_tx", t_tx, "t_meas", t_meas, "packet_age", packet_age)
            print("p_i_meas", p_i_meas, "p_j_meas", p_j_meas)
            print("q_sender", q_j)
            print("R_corr_eff", R_corr_eff)
            return

        K = (P_ii_meas @ H_i.T) / S

        x_i_post = x_i_meas.copy()
        x_i_post = x_i_post + K[:, 0] * nu
        x_i_post[3] = self._wrap_deg(x_i_post[3])

        I = np.eye(6)
        KH = K @ H_i
        R_mat = np.array([[R_eff]], dtype=float)
        P_ii_post = (I - KH) @ P_ii_meas @ (I - KH).T + K @ R_mat @ K.T
        P_ii_post = self._symmetrize(P_ii_post)

        x_replay = x_i_post.copy()
        P_replay = P_ii_post.copy()
        t_replay = float(t_meas)

        for k in range(hist_idx + 1, len(state.history)):
            entry = state.history[k]
            if entry.get("kind") != "snapshot":
                continue

            t_k = float(entry["t"])
            if t_k <= t_meas:
                continue
            if t_k > t_now + 1e-12:
                break

            dt = t_k - t_replay
            if dt < -1e-12:
                continue

            if dt > 0.0:
                x_replay, P_replay = self._propagate_cov(
                    x=x_replay,
                    P=P_replay,
                    dt=dt,
                )
                t_replay = t_k

            x_replay, P_replay = self._apply_local_measurements_to_state(
                x_replay,
                P_replay,
                z_depth=float(entry["z_depth"]),
                z_heading=float(entry["z_heading"]),
                z_body_vel=np.asarray(entry["z_body_vel"], dtype=float),
            )

            if "z_xy" in entry:
                R_xy = entry.get("R_xy", self.R_surface_pos)
                x_replay, P_replay = self._apply_surface_xy_measurement_to_state(
                    x_replay,
                    P_replay,
                    z_xy=np.asarray(entry["z_xy"], dtype=float),
                    R_xy=np.asarray(R_xy, dtype=float),
                )

        dt_tail = float(t_now - t_replay)
        if dt_tail > 0.0:
            x_replay, P_replay = self._propagate_cov(
                x=x_replay,
                P=P_replay,
                dt=dt_tail,
            )

        state.x = x_replay
        state.P = P_replay
        state.t = float(t_now)
        state.x[3] = self._wrap_deg(state.x[3])
        state.last_coop_update = float(t_now)
        state.quality = float(np.trace(state.P[:3, :3]))

        self.coop_update_log.append(
            {
                "t": float(t_now),
                "receiver": receiver_name,
                "sender": sender_name,
                "accepted": 1,
                "range": float(z_range),
                "r_hat": float(r_hat),
                "nu": float(nu),
                "nis": float(nis),
                "nis_gate": float(self.nis_gate),
                "S": float(S),
                "R_eff": float(R_eff),
                "R_sender": float(R_sender),
                "R_delay": float(R_delay),
                "R_corr_eff": float(R_corr_eff),
                "q_sender": float(q_j) if np.isfinite(q_j) else np.nan,
                "packet_age": float(packet_age),
                "t_tx_payload": float(t_tx),
                "t_meas": float(t_meas),
                "dt_sender": float(dt_j),
                "dt_receiver": float(t_now - t_meas),
                "sender_x_meas": float(p_j_meas[0]),
                "sender_y_meas": float(p_j_meas[1]),
                "sender_z_meas": float(p_j_meas[2]),
                "receiver_x_meas": float(x_i_meas[0]),
                "receiver_y_meas": float(x_i_meas[1]),
                "receiver_z_meas": float(x_i_meas[2]),
                "receiver_x_post": float(state.x[0]),
                "receiver_y_post": float(state.x[1]),
                "receiver_z_post": float(state.x[2]),
                "traceP_pre": float(np.trace(P_pre_now[:3, :3])),
                "traceP_post": float(np.trace(state.P[:3, :3])),
            }
        )

    # ==========================================================
    # Adaptive cooperative noise
    # ==========================================================

    def _adaptive_R_corr(self, q_j):
        """
        Adapt cooperative correlation/noise inflation using sender navigation quality.

        Low q_j means the sender is poorly localized, therefore the cooperative
        range update should be trusted less.
        """
        if not self.R_corr_q_adapt:
            return float(self.R_corr)

        if not np.isfinite(q_j) or q_j <= 0.0:
            return float(self.R_corr_max)

        scale = 1.0 + self.R_corr_q_beta * (
            self.R_corr_q_ref / (q_j + 1e-12)
        )

        R_corr_eff = self.R_corr * scale
        return float(np.clip(R_corr_eff, self.R_corr_min, self.R_corr_max))

    # ==========================================================
    # Transition model
    # ==========================================================

    def _propagate_cov(self, x, P, dt):
        x = np.asarray(x, dtype=float).copy()
        P = np.asarray(P, dtype=float).copy()

        if dt <= 0.0:
            return x, P

        F = self._transition_jacobian(x, dt)
        Q = np.diag(self.Q_diag * dt)

        x_next = self._propagate_state(x, dt)
        P_next = self._symmetrize(F @ P @ F.T + Q)

        return x_next, P_next

    def _propagate_state(self, x, dt):
        x_next = np.asarray(x, dtype=float).copy()

        psi = float(x_next[3])
        vel_body = x_next[4:6]
        vel_ned = self._body_to_ned_2d(vel_body, psi)

        x_next[0] += vel_ned[0] * dt
        x_next[1] += vel_ned[1] * dt
        x_next[3] = self._wrap_deg(x_next[3])

        return x_next

    def _transition_jacobian(self, x, dt):
        return self._constvel_jacobian(x, dt)

    # ==========================================================
    # Velocity helpers
    # ==========================================================

    def _is_surface_agent(self, agent):
        return agent.name in self.surface_agents

    def _get_body_velocity_measurement(self, agent, sim):
        return self._get_body_velocity_from_agent(agent, sim)

    @staticmethod
    def _get_body_velocity_from_agent(agent, sim):
        if hasattr(agent, "emulated_velocities"):
            return np.asarray(agent.emulated_velocities, dtype=float).reshape(2).copy()
        return np.zeros(2, dtype=float)

    @staticmethod
    def _body_to_ned_2d(v_body, psi_deg):
        psi = np.deg2rad(psi_deg)
        sin_psi = np.sin(psi)
        cos_psi = np.cos(psi)

        R_body_to_ned = np.array(
            [
                [cos_psi, sin_psi],
                [sin_psi, -cos_psi],
            ],
            dtype=float,
        )
        return R_body_to_ned @ np.asarray(v_body, dtype=float).reshape(2)

    # ==========================================================
    # EKF helpers
    # ==========================================================

    def _constvel_jacobian(self, x, dt):
        psi = np.deg2rad(x[3])
        u = float(x[4])
        v = float(x[5])

        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        deg_to_rad = np.pi / 180.0

        F = np.eye(6)
        F[0, 3] = dt * (-sin_psi * u + cos_psi * v) * deg_to_rad
        F[0, 4] = dt * cos_psi
        F[0, 5] = dt * sin_psi
        F[1, 3] = dt * (cos_psi * u + sin_psi * v) * deg_to_rad
        F[1, 4] = dt * sin_psi
        F[1, 5] = -dt * cos_psi
        return F

    def _ekf_update_linear(self, state, z, h, H, R, angle_idx=None):
        z = np.asarray(z, dtype=float).reshape(-1)
        h = np.asarray(h, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)

        nu = z - h
        if angle_idx is not None:
            nu[angle_idx] = self._wrap_deg(nu[angle_idx])

        S = self._symmetrize(H @ state.P @ H.T + R)

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        K = state.P @ H.T @ S_inv

        state.x = state.x + K @ nu
        state.x[3] = self._wrap_deg(state.x[3])

        I = np.eye(state.P.shape[0])
        KH = K @ H
        state.P = (I - KH) @ state.P @ (I - KH).T + K @ R @ K.T
        state.P = self._symmetrize(state.P)

    def _apply_local_measurements(self, state, z_depth, z_heading, z_body_vel):
        H_depth = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=float)
        self._ekf_update_linear(
            state,
            z=np.asarray(z_depth, dtype=float),
            h=np.array([state.x[2]], dtype=float),
            H=H_depth,
            R=np.array([[self.R_depth]], dtype=float),
            angle_idx=None,
        )

        H_heading = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]], dtype=float)
        self._ekf_update_linear(
            state,
            z=np.array([float(z_heading)], dtype=float),
            h=np.array([state.x[3]], dtype=float),
            H=H_heading,
            R=np.array([[self.R_heading]], dtype=float),
            angle_idx=0,
        )

        H_body_vel = np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        self._ekf_update_linear(
            state,
            z=np.asarray(z_body_vel, dtype=float),
            h=state.x[4:6].copy(),
            H=H_body_vel,
            R=self.R_body_vel,
            angle_idx=None,
        )

    def _apply_local_measurements_to_state(self, x, P, z_depth, z_heading, z_body_vel):
        state = NavState(
            x=np.asarray(x, dtype=float).copy(),
            P=np.asarray(P, dtype=float).copy(),
            t=0.0,
        )
        self._apply_local_measurements(
            state,
            z_depth=np.array([float(z_depth)], dtype=float),
            z_heading=float(z_heading),
            z_body_vel=np.asarray(z_body_vel, dtype=float),
        )
        return state.x.copy(), state.P.copy()

    def _apply_surface_xy_measurement_to_state(self, x, P, z_xy, R_xy):
        state = NavState(
            x=np.asarray(x, dtype=float).copy(),
            P=np.asarray(P, dtype=float).copy(),
            t=0.0,
        )

        H_xy = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        self._ekf_update_linear(
            state,
            z=np.asarray(z_xy, dtype=float),
            h=state.x[0:2].copy(),
            H=H_xy,
            R=np.asarray(R_xy, dtype=float),
            angle_idx=None,
        )

        return state.x.copy(), state.P.copy()

    def _propagate_sender_to_meas(self, x_j_tx, P_j_tx, dt_j):
        x_j_meas = np.asarray(x_j_tx, dtype=float).copy()

        vel_j_ned = self._body_to_ned_2d(x_j_meas[4:6], x_j_meas[3])
        x_j_meas[0] += vel_j_ned[0] * dt_j
        x_j_meas[1] += vel_j_ned[1] * dt_j
        x_j_meas[3] = self._wrap_deg(x_j_meas[3])

        F_j = self._constvel_jacobian(x_j_tx, dt_j)
        P_jj_tx = np.zeros((6, 6), dtype=float)
        P_jj_tx[:3, :3] = P_j_tx

        Q_j = np.diag(self.Q_diag * max(dt_j, 0.0))
        P_jj_meas = self._symmetrize(F_j @ P_jj_tx @ F_j.T + Q_j)

        return x_j_meas, P_jj_meas

    # ==========================================================
    # History helpers
    # ==========================================================

    def _append_history_snapshot(
        self,
        state,
        t,
        x,
        P,
        z_depth,
        z_heading,
        z_body_vel,
        z_xy=None,
        R_xy=None,
    ):
        entry = {
            "kind": "snapshot",
            "t": float(t),
            "x": np.asarray(x, dtype=float).copy(),
            "P": np.asarray(P, dtype=float).copy(),
            "z_depth": float(z_depth),
            "z_heading": float(z_heading),
            "z_body_vel": np.asarray(z_body_vel, dtype=float).copy(),
        }

        if z_xy is not None:
            entry["z_xy"] = np.asarray(z_xy, dtype=float).copy()

        if R_xy is not None:
            entry["R_xy"] = np.asarray(R_xy, dtype=float).copy()

        state.history.append(entry)
        self._trim_history(state)

    def _trim_history(self, state):
        if len(state.history) > self.history_length:
            state.history[:] = state.history[-self.history_length:]

    @staticmethod
    def _get_history_index_before_or_equal(state, t_query):
        idx = None
        for k, entry in enumerate(state.history):
            if float(entry.get("t", -np.inf)) <= float(t_query) + 1e-12:
                idx = k
            else:
                break
        return idx

    @staticmethod
    def _get_nearest_snapshot_index_before(state, start_idx):
        for k in range(start_idx, -1, -1):
            if state.history[k].get("kind") == "snapshot":
                return k
        return None

    # ==========================================================
    # Writeback helpers
    # ==========================================================

    def _writeback_all(self, sim):
        for agent in sim.agents.values():
            state = self.filters[agent.name]

            agent.nav_state = state
            agent.est_pos = state.x[:3].copy()
            agent.est_heading = float(state.x[3])
            agent.est_body_vel = state.x[4:6].copy()
            agent.est_cov = state.P.copy()

            agent.nav_info = {
                "traceP_pos": float(np.trace(state.P[:3, :3])),
                "detP_pos": float(np.linalg.det(state.P[:3, :3])),
                "traceP_vel": float(np.trace(state.P[4:6, 4:6])),
                "last_local_update": float(state.last_local_update),
                "last_coop_update": float(state.last_coop_update),
                "quality": float(state.quality),
                "t": float(state.t),
                "nu": float(self.last_range_residual),
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
            P = np.asarray(cov_payload, dtype=float)
        except Exception:
            return None

        if P.ndim == 0:
            variance = float(P)
            if variance < 0.0:
                return None
            return np.eye(3) * variance

        if P.shape == (3,):
            if np.any(P < 0.0):
                return None
            return np.diag(P)

        if P.shape == (3, 3):
            P = 0.5 * (P + P.T)
            if np.any(np.linalg.eigvalsh(P) < -1e-10):
                return None
            return P

        return None