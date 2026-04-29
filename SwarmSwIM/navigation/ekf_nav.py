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

    last_coop_meas_time: dict = field(default_factory=dict)
    history: list = field(default_factory=list)


class EKFNavFilter(BaseNavFilter):
    """
    Distributed Cooperative EKF for SwarmSwIM.

    State
    -----
    x = [x, y, z, psi, u, v]^T
    """

    def __init__(
        self,
        Q_diag=(0.01, 0.01, 0.0001, 0.03, 0.01, 0.02),
        P0_diag=(0.25, 0.25, 1e-4, 9.0, 0.01, 0.01),
        R_depth=1e-6,
        R_heading_deg=4.0,
        R_body_vel_diag=np.array([0.003, 0.003]),
        sigma_init_pos=np.array([0.25, 0.25]),
        R_range=0.01,
        R_coop_updates=0.5,
        alpha=1.0,
        min_range=1e-2,

        sigma_rel_speed=0.2,
        history_length=1000,
        writeback=True,
        keep_history=True,
        rng_seed=50,
        surface_agents=("A04", "A02"),
        R_surface_pos_diag=(0.04, 0.04),
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.P0_diag = np.asarray(P0_diag, dtype=float)

        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))
        self.R_range = float(R_range)
        self.R_coop_updates = float(R_coop_updates)


        self.sigma_init_pos = np.diag(np.asarray(sigma_init_pos, dtype=float))

        self.alpha = float(alpha)
        self.min_range = float(min_range)
        self.sigma_rel_speed = float(sigma_rel_speed)

        self.history_length = int(history_length)
        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.surface_agents = set(surface_agents)
        self.R_surface_pos = np.diag(np.asarray(R_surface_pos_diag, dtype=float))

        self.rng = np.random.default_rng(rng_seed)

        self.coop_update_log = []
        self.nu = np.nan

    # ==========================================================
    # Base hooks
    # ==========================================================

    def _init_filter(self, agent):
        pos_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.sigma_init_pos)),
            size=2
        )

        vel0 = self._get_body_velocity_measurement(agent, None)

        x0 = np.array([
            float(agent.pos[0]) + pos_noise[0],
            float(agent.pos[1]) + pos_noise[1],
            float(agent.pos[2]),
            float(agent.psi),
            float(vel0[0]),
            float(vel0[1]),
        ], dtype=float)

        P0 = np.diag(self.P0_diag)

        st = NavState(
            x=x0,
            P=P0,
            t=0.0,
            last_local_update=0.0,
            last_coop_update=0.0,
            quality=float(np.trace(P0[:3, :3])),
        )

        if self.keep_history:
            self._append_history_snapshot(
                st=st,
                t=0.0,
                x=st.x,
                P=st.P,
                meas_depth=float(agent.measured_depth),
                meas_heading=float(agent.measured_heading),
                meas_body_vel=np.asarray(vel0, dtype=float),
            )

        return st

    def predict(self, agent, sim):
        st = self.filters[agent.name]
        dt = float(sim.time - st.t)
        if dt <= 0.0:
            return

        F = self._transition_jacobian(st.x, dt)
        x_pred = self._propagate_state(st.x, dt)

        Q = np.diag(self.Q_diag * dt)

        st.x = x_pred
        st.P = self._symmetrize(F @ st.P @ F.T + Q)
        st.t = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))

        if self.keep_history:
            st.history.append({
                "kind": "predict_trace",
                "t": float(sim.time),
                "x": st.x.copy(),
                "P": st.P.copy(),
            })
            self._trim_history(st)

    def update_local(self, agent, sim):
        st = self.filters[agent.name]

        z_depth = np.array([float(agent.measured_depth)], dtype=float)
        z_psi = float(agent.measured_heading)
        z_vel = self._get_body_velocity_measurement(agent, sim)

        self._apply_local_measurements(st, z_depth, z_psi, z_vel)

        if self._is_surface_agent(agent):
            pos_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.sigma_init_pos)),
            size=2
            )
            z_xy = np.array([float(agent.pos[0])+pos_noise[0], float(agent.pos[1])+pos_noise[0]], dtype=float)
            H_xy = np.array([
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            ], dtype=float)

            self._ekf_update_linear(
                st,
                z=z_xy,
                h=st.x[0:2].copy(),
                H=H_xy,
                R=self.R_surface_pos,
                angle_idx=None,
            )

        st.last_local_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))

        if self.keep_history:
            self._append_history_snapshot(
                st=st,
                t=float(sim.time),
                x=st.x,
                P=st.P,
                meas_depth=float(z_depth[0]),
                meas_heading=float(z_psi),
                meas_body_vel=np.asarray(z_vel, dtype=float),
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

            st = self.filters[receiver_name]
            meas = receiver.AcousticRange[sender_name]
            payload = msg.payload

            if payload is None:
                continue
            if payload.get("type") != "nav":#(process only if nav payload present)
                continue
            if "pos" not in payload or "cov" not in payload or "body_vel" not in payload:
                continue
            if "pos" not in payload or "cov" not in payload or "body_vel" not in payload:
                continue

            t_meas = meas.get("t_meas", None)
            if t_meas is None:
                continue
            t_meas = float(t_meas)

            last_t_meas = st.last_coop_meas_time.get(sender_name, -np.inf)
            if t_meas <= last_t_meas:
                continue

            p_j = np.asarray(payload["pos"], dtype=float).reshape(-1)
            if p_j.size != 3:
                continue

            v_j = np.asarray(payload["body_vel"], dtype=float).reshape(-1)
            if v_j.size != 2:
                continue

            psi_j = float(payload.get("heading", np.nan))
            if not np.isfinite(psi_j):
                continue

            P_j = self._parse_sender_cov(payload["cov"])
            if P_j is None:
                continue

            t_tx = float(payload.get("tx_time", meas.get("t_tx", sim.time)))
            staleness = max(0.0, float(sim.time) - t_tx)
            z = float(meas["range"])

            self._update_cooperative_range(
                receiver_name=receiver_name,
                sender_name=sender_name,
                p_j=p_j,
                psi_j=psi_j,
                v_j=v_j,
                P_j=P_j,
                z=z,
                staleness=staleness,
                t_now=float(sim.time),
                t_tx=t_tx,
                t_meas=t_meas,
                sim=sim,
            )
            st.last_coop_meas_time[sender_name] = t_meas

        if self.writeback:
            self._writeback_all(sim)

    # ==========================================================
    # Cooperative range update
    # ==========================================================

    def _update_cooperative_range(
        self,
        receiver_name,
        sender_name,
        p_j,
        psi_j,
        v_j,
        P_j,
        z,
        staleness,
        t_now,
        t_tx=np.nan,
        t_meas=np.nan,
        sim=None,
    ):
        st = self.filters[receiver_name]

        x_pre_now = st.x.copy()
        P_pre_now = st.P.copy()

        hist_idx = self._get_history_index_before_or_equal(st, t_meas)
        if hist_idx is None:
            return

        hist_entry = st.history[hist_idx]
        if hist_entry.get("kind") != "snapshot":
            hist_idx = self._get_nearest_snapshot_index_before(st, hist_idx)
            if hist_idx is None:
                return
            hist_entry = st.history[hist_idx]

        t_hist = float(hist_entry["t"])
        x_rx_hist = hist_entry["x"].copy()
        P_rx_hist = hist_entry["P"].copy()

        dt_hist_to_meas = max(0.0, float(t_meas) - t_hist)
        x_rx_meas, P_rx_meas, _ = self._propagate_cov(
            x=x_rx_hist,
            P=P_rx_hist,
            dt=dt_hist_to_meas,
        )

        dt_sender = 0.0
        if np.isfinite(t_tx) and np.isfinite(t_meas):
            dt_sender = max(0.0, float(t_meas) - float(t_tx))

        xj_tx = np.array([
            float(p_j[0]),
            float(p_j[1]),
            float(p_j[2]),
            float(psi_j),
            float(v_j[0]),
            float(v_j[1]),
        ], dtype=float)

        xj_meas, Pj6_meas = self._propagate_sender_to_meas(xj_tx, P_j, dt_sender)
        p_j_meas = xj_meas[:3].copy()
        P_j_meas = Pj6_meas[:3, :3]

        p_i_meas = x_rx_meas[:3].copy()
        diff = p_i_meas - p_j_meas
        r_hat = float(np.linalg.norm(diff))
        if r_hat < self.min_range:
            return

        H_i = np.array([[
            diff[0] / r_hat,
            diff[1] / r_hat,
            diff[2] / r_hat,
            0.0, 0.0, 0.0
        ]], dtype=float)

        H_j = np.array([[
            -diff[0] / r_hat,
            -diff[1] / r_hat,
            -diff[2] / r_hat
        ]], dtype=float)

        alpha_eff = 0.2 if sender_name in self.surface_agents else self.alpha
        R_coop_updates = 0.1 if sender_name in self.surface_agents else self.R_coop_updates
        R_sender = alpha_eff * float((H_j @ P_j_meas @ H_j.T)[0, 0])
        R_delay = float((self.sigma_rel_speed * staleness) ** 2)
        R_eff = float(self.R_range + R_sender + R_delay + R_coop_updates)
        R_eff = max(R_eff, 1e-12)

        nu = float(z - r_hat)
        self.nu = nu

        S = float((H_i @ P_rx_meas @ H_i.T)[0, 0] + R_eff)
        if S <= 0.0:
            return

        K = (P_rx_meas @ H_i.T) / S

        x_post_meas = x_rx_meas.copy()
        x_post_meas = x_post_meas + (K[:, 0] * nu)
        x_post_meas[3] = self._wrap_deg(x_post_meas[3])

        I = np.eye(6)
        KH = K @ H_i
        Rm = np.array([[R_eff]], dtype=float)
        P_post_meas = (I - KH) @ P_rx_meas @ (I - KH).T + K @ Rm @ K.T
        P_post_meas = self._symmetrize(P_post_meas)

        nis = float((nu ** 2) / S)

        x_replay = x_post_meas.copy()
        P_replay = P_post_meas.copy()
        t_replay = float(t_meas)

        for k in range(hist_idx + 1, len(st.history)):
            entry = st.history[k]
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
                x_replay, P_replay, _ = self._propagate_cov(
                    x=x_replay,
                    P=P_replay,
                    dt=dt,
                )
                t_replay = t_k

            x_replay, P_replay = self._apply_local_measurements_to_state(
                x_replay,
                P_replay,
                meas_depth=float(entry["meas_depth"]),
                meas_heading=float(entry["meas_heading"]),
                meas_body_vel=np.asarray(entry["meas_body_vel"], dtype=float),
            )

        dt_tail = float(t_now - t_replay)
        if dt_tail > 0.0:
            x_replay, P_replay, _ = self._propagate_cov(
                x=x_replay,
                P=P_replay,
                dt=dt_tail,
            )

        st.x = x_replay
        st.P = P_replay
        st.t = float(t_now)
        st.x[3] = self._wrap_deg(st.x[3])
        st.last_coop_update = float(t_now)
        st.quality = float(np.trace(st.P[:3, :3]))

        self.coop_update_log.append({
            "t": float(t_now),
            "receiver": receiver_name,
            "sender": sender_name,
            "accepted": 1,
            "range": float(z),
            "r_hat": float(r_hat),
            "nu": float(nu),
            "nis": float(nis),
            "S": float(S),
            "R_eff": float(R_eff),
            "packet_age": float(staleness),
            "t_tx_payload": float(t_tx),
            "t_meas": float(t_meas),
            "dt_sender": float(dt_sender),
            "dt_receiver": float(t_now - t_meas),
            "sender_x_meas": float(p_j_meas[0]),
            "sender_y_meas": float(p_j_meas[1]),
            "sender_z_meas": float(p_j_meas[2]),
            "receiver_x_meas": float(x_rx_meas[0]),
            "receiver_y_meas": float(x_rx_meas[1]),
            "receiver_z_meas": float(x_rx_meas[2]),
            "receiver_x_post": float(st.x[0]),
            "receiver_y_post": float(st.x[1]),
            "receiver_z_post": float(st.x[2]),
            "traceP_pre": float(np.trace(P_pre_now[:3, :3])),
            "traceP_post": float(np.trace(st.P[:3, :3])),
        })

    # ==========================================================
    # Transition model
    # ==========================================================


    def _propagate_cov(self, x, P, dt):
        if dt <= 0.0:
            return (
                np.asarray(x, dtype=float).copy(),
                np.asarray(P, dtype=float).copy(),
                None,
            )

        x = np.asarray(x, dtype=float).copy()
        P = np.asarray(P, dtype=float).copy()

        F = self._transition_jacobian(x, dt)
        x_next = self._propagate_state(x, dt)
        Q = np.diag(self.Q_diag * dt)
        P_next = self._symmetrize(F @ P @ F.T + Q)

        return x_next, P_next, None

    def _propagate_state(self, x, dt):
        x = np.asarray(x, dtype=float).copy()

        psi = float(x[3])
        u = float(x[4])
        v = float(x[5])

        vel_ned = self._body_to_ned_2d(np.array([u, v], dtype=float), psi)

        x[0] += vel_ned[0] * dt
        x[1] += vel_ned[1] * dt
        x[3] = self._wrap_deg(x[3])

        return x

    def _transition_jacobian(self, x, dt):
        return self._constvel_jacobian(x, dt)

    # ==========================================================
    # Velocity helpers
    # ==========================================================

    def _is_surface_agent(self, agent):
        return agent.name in self.surface_agents

    def _get_body_velocity_measurement(self, agent, sim):
        body_vel = self._get_body_velocity_from_agent(agent, sim)
        return body_vel

    def _get_body_velocity_from_agent(self, agent, sim):
        if hasattr(agent, "emulated_velocities"):
            vel = np.asarray(agent.emulated_velocities, dtype=float).reshape(2)
            return vel.copy()
        return np.zeros(2, dtype=float)

    @staticmethod
    def _body_to_ned_2d(v_body, psi_deg):
        psi = np.deg2rad(psi_deg)
        sinpsi = np.sin(psi)
        cospsi = np.cos(psi)
        R_mat = np.array([[cospsi, sinpsi], [sinpsi, -cospsi]], dtype=float)
        return R_mat @ np.asarray(v_body, dtype=float).reshape(2)

    # ==========================================================
    # EKF helpers
    # ==========================================================

    def _constvel_jacobian(self, x, dt):
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

    def _ekf_update_linear(self, st, z, h, H, R, angle_idx=None):
        z = np.asarray(z, dtype=float).reshape(-1)
        h = np.asarray(h, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)

        nu = z - h
        if angle_idx is not None:
            nu[angle_idx] = self._wrap_deg(nu[angle_idx])

        S = self._symmetrize(H @ st.P @ H.T + R)

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

    def _apply_local_measurements(self, st, z_depth, z_psi, z_vel):
        H_depth = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=float)
        self._ekf_update_linear(
            st,
            z=np.asarray(z_depth, dtype=float),
            h=np.array([st.x[2]], dtype=float),
            H=H_depth,
            R=np.array([[self.R_depth]], dtype=float),
            angle_idx=None
        )

        H_psi = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]], dtype=float)
        self._ekf_update_linear(
            st,
            z=np.array([float(z_psi)], dtype=float),
            h=np.array([st.x[3]], dtype=float),
            H=H_psi,
            R=np.array([[self.R_heading]], dtype=float),
            angle_idx=0
        )

        H_vel = np.array([
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
        self._ekf_update_linear(
            st,
            z=np.asarray(z_vel, dtype=float),
            h=st.x[4:6].copy(),
            H=H_vel,
            R=self.R_body_vel,
            angle_idx=None
        )

    def _apply_local_measurements_to_state(self, x, P, meas_depth, meas_heading, meas_body_vel):
        st_tmp = NavState(
            x=np.asarray(x, dtype=float).copy(),
            P=np.asarray(P, dtype=float).copy(),
            t=0.0
        )
        self._apply_local_measurements(
            st_tmp,
            z_depth=np.array([float(meas_depth)], dtype=float),
            z_psi=float(meas_heading),
            z_vel=np.asarray(meas_body_vel, dtype=float),
        )
        return st_tmp.x.copy(), st_tmp.P.copy()

    def _propagate_sender_to_meas(self, xj_tx, P_j, dt_sender):
        xj_meas = np.asarray(xj_tx, dtype=float).copy()
        vel_ned = self._body_to_ned_2d(xj_meas[4:6], xj_meas[3])
        xj_meas[0] += vel_ned[0] * dt_sender
        xj_meas[1] += vel_ned[1] * dt_sender
        xj_meas[3] = self._wrap_deg(xj_meas[3])

        Fj = self._constvel_jacobian(xj_tx, dt_sender)
        Pj6 = np.zeros((6, 6), dtype=float)
        Pj6[:3, :3] = P_j
        Qj = np.diag(self.Q_diag * max(dt_sender, 0.0))
        Pj6_meas = self._symmetrize(Fj @ Pj6 @ Fj.T + Qj)
        return xj_meas, Pj6_meas

    # ==========================================================
    # History helpers
    # ==========================================================

    def _append_history_snapshot(self, st, t, x, P, meas_depth, meas_heading, meas_body_vel):
        st.history.append({
            "kind": "snapshot",
            "t": float(t),
            "x": np.asarray(x, dtype=float).copy(),
            "P": np.asarray(P, dtype=float).copy(),
            "meas_depth": float(meas_depth),
            "meas_heading": float(meas_heading),
            "meas_body_vel": np.asarray(meas_body_vel, dtype=float).copy(),
        })
        self._trim_history(st)

    def _trim_history(self, st):
        if len(st.history) > self.history_length:
            st.history[:] = st.history[-self.history_length:]

    def _get_history_index_before_or_equal(self, st, t_query):
        idx = None
        for k, entry in enumerate(st.history):
            if float(entry.get("t", -np.inf)) <= float(t_query) + 1e-12:
                idx = k
            else:
                break
        return idx

    def _get_nearest_snapshot_index_before(self, st, start_idx):
        for k in range(start_idx, -1, -1):
            if st.history[k].get("kind") == "snapshot":
                return k
        return None

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
                "nu": float(self.nu),
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