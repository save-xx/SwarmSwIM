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

    # last processed cooperative measurement timestamp per sender
    last_coop_meas_time: dict = field(default_factory=dict)

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
    """

    def __init__(
        self,
        Q_diag = (0.003, 0.003, 0.0001, 0.03, 0.001, 0.01),
        P0_diag=(0.25, 0.25, 1e-4, 9.0, 0.01, 0.01),
        R_depth=1e-6,
        R_heading_deg=4.0,                      # variance [deg^2], std = 2 deg
        R_body_vel_diag=(0.01, 0.01),          # variance [(m/s)^2], std = 0.1
        sigma_body_vel=np.array([0.01, 0.01]), # variance used to synthesize vel measurement noise
        sigma_init_pos=np.array([0.25, 0.25]), # variance [m^2], std = 0.5
        
        
        
        R_range=0.01,                          # variance [m^2], std = 0.1
        R_coop_updates = 0.98,
        alpha=5.0,
        max_packet_age=np.inf,
        min_range=1e-6,
        nis_gate=6.63,                         # 99% for 1 DoF
        sigma_rel_speed=0.2,                   # m/s, maps staleness to equivalent range std
        history_length=1000,
        writeback=True,
        keep_history=True,
        rng_seed=50,
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))
        self.sigma_body_vel = np.diag(np.asarray(sigma_body_vel, dtype=float))
        self.sigma_init_pos = np.diag(np.asarray(sigma_init_pos, dtype=float))
        self.R_range = float(R_range)
        self.R_coop_updates = float(R_coop_updates)
        self.P0_diag = np.asarray(P0_diag, dtype=float)
        self.alpha = float(alpha)
        self.max_packet_age = float(max_packet_age)
        self.min_range = float(min_range)
        self.nis_gate = float(nis_gate)
        self.sigma_rel_speed = float(sigma_rel_speed)

        self.history_length = int(history_length)

        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.rng = np.random.default_rng(rng_seed)

        # accepted/rejected EKF cooperative updates
        self.coop_update_log = []

        self.nu = np.nan

    # ==========================================================
    # Base hooks
    # ==========================================================

    def _init_filter(self, agent):
        vel0 = self._get_initial_body_velocity(agent)
        pos_init_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.sigma_init_pos)),
            size=2
        )

        x0 = np.array([
            float(agent.pos[0]) + pos_init_noise[0],
            float(agent.pos[1]) + pos_init_noise[1],
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

        st = NavState(
            x=x0,
            P=P0,
            t=0.0,
            last_local_update=0.0,
            last_coop_update=0.0,
            quality=float(np.trace(P0[:3, :3])),
            model_agent=model_agent,
            prev_true_pos=np.asarray(agent.pos[:2], dtype=float).copy()
        )

        if self.keep_history:
            self._append_history_snapshot(
                st=st,
                t=0.0,
                x=st.x,
                P=st.P,
                model_agent=st.model_agent,
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

        F = self._numerical_transition_jacobian(agent, sim, st)
        x_pred, model_next = self._propagate_shadow(agent, sim, st.model_agent, st.x)

        Q = np.diag(self.Q_diag * dt)

        st.x = x_pred
        st.P = self._symmetrize(F @ st.P @ F.T + Q)
        st.t = float(sim.time)
        st.model_agent = model_next
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

        st.last_local_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))
        st.prev_true_pos = np.asarray(agent.pos[:2], dtype=float).copy()

        if self.keep_history:
            self._append_history_snapshot(
                st=st,
                t=float(sim.time),
                x=st.x,
                P=st.P,
                model_agent=st.model_agent,
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
            if "pos" not in payload or "cov" not in payload:
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

            P_j = self._parse_sender_cov(payload["cov"])
            if P_j is None:
                continue

            t_tx = float(payload.get("tx_time", meas.get("t_tx", sim.time)))
            staleness = max(0.0, float(sim.time) - t_tx)

            if staleness > self.max_packet_age:
                self._log_coop_reject(
                    receiver_name=receiver_name,
                    sender_name=sender_name,
                    reject_reason="max_packet_age",
                    z=meas.get("range", np.nan),
                    r_hat=np.nan,
                    nu=np.nan,
                    nis=np.nan,
                    S=np.nan,
                    R_eff=np.nan,
                    packet_age=staleness,
                    t_now=sim.time,
                    t_tx=t_tx,
                    t_meas=t_meas,
                    dt_sender=np.nan,
                    dt_receiver=np.nan,
                    sender_xyz=(np.nan, np.nan, np.nan),
                    receiver_xyz=(np.nan, np.nan, np.nan),
                    receiver_xyz_post=tuple(st.x[:3]),
                    traceP_pre=float(np.trace(st.P[:3, :3])),
                    traceP_post=float(np.trace(st.P[:3, :3])),
                )
                continue

            z = float(meas["range"])

            self._update_cooperative_range(
                receiver_name=receiver_name,
                sender_name=sender_name,
                p_j=p_j,
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
    # Cooperative range update with delayed-state replay
    # ==========================================================

    def _update_cooperative_range(
        self,
        receiver_name,
        sender_name,
        p_j,
        P_j,
        z,
        staleness,
        t_now,
        t_tx=np.nan,
        t_meas=np.nan,
        sim=None,
    ):
        DEBUG_AGENT = "A01"
        debug = (receiver_name == DEBUG_AGENT)
        debug = False
        st = self.filters[receiver_name]

        x_pre_now = st.x.copy()
        P_pre_now = st.P.copy()

        # -----------------------------
        # locate receiver state before t_meas
        # -----------------------------
        hist_idx = self._get_history_index_before_or_equal(st, t_meas)
        if hist_idx is None:
            self._log_coop_reject(
                receiver_name=receiver_name,
                sender_name=sender_name,
                reject_reason="no_history",
                z=z,
                r_hat=np.nan,
                nu=np.nan,
                nis=np.nan,
                S=np.nan,
                R_eff=np.nan,
                packet_age=staleness,
                t_now=t_now,
                t_tx=t_tx,
                t_meas=t_meas,
                dt_sender=np.nan,
                dt_receiver=np.nan,
                sender_xyz=(np.nan, np.nan, np.nan),
                receiver_xyz=(np.nan, np.nan, np.nan),
                receiver_xyz_post=tuple(x_pre_now[:3]),
                traceP_pre=float(np.trace(P_pre_now[:3, :3])),
                traceP_post=float(np.trace(P_pre_now[:3, :3])),
            )
            return

        hist_entry = st.history[hist_idx]
        if hist_entry.get("kind") != "snapshot":
            hist_idx = self._get_nearest_snapshot_index_before(st, hist_idx)
            if hist_idx is None:
                self._log_coop_reject(
                    receiver_name=receiver_name,
                    sender_name=sender_name,
                    reject_reason="no_snapshot_history",
                    z=z,
                    r_hat=np.nan,
                    nu=np.nan,
                    nis=np.nan,
                    S=np.nan,
                    R_eff=np.nan,
                    packet_age=staleness,
                    t_now=t_now,
                    t_tx=t_tx,
                    t_meas=t_meas,
                    dt_sender=np.nan,
                    dt_receiver=np.nan,
                    sender_xyz=(np.nan, np.nan, np.nan),
                    receiver_xyz=(np.nan, np.nan, np.nan),
                    receiver_xyz_post=tuple(x_pre_now[:3]),
                    traceP_pre=float(np.trace(P_pre_now[:3, :3])),
                    traceP_post=float(np.trace(P_pre_now[:3, :3])),
                )
                return
            hist_entry = st.history[hist_idx]

        t_hist = float(hist_entry["t"])
        x_rx_hist = hist_entry["x"].copy()
        P_rx_hist = hist_entry["P"].copy()
        model_hist = copy.deepcopy(hist_entry["model_agent"])

        # propagate receiver from stored state to exact t_meas
        dt_hist_to_meas = max(0.0, float(t_meas) - t_hist)
        x_rx_meas, P_rx_meas, model_rx_meas = self._propagate_shadow_mean_cov(
            receiver_name=receiver_name,
            x=x_rx_hist,
            P=P_rx_hist,
            model_agent=model_hist,
            dt=dt_hist_to_meas,
            sim=sim,
        )

        # -----------------------------
        # sender state at t_meas
        # -----------------------------
        dt_sender = 0.0
        if np.isfinite(t_tx) and np.isfinite(t_meas):
            dt_sender = max(0.0, float(t_meas) - float(t_tx))


        dt_sender 

        # current simplified scenario: true known sender motion
        xj_tx = np.array([p_j[0], p_j[1], p_j[2], 180.0, 0.5, 0.0], dtype=float)
        xj_meas, Pj6_meas = self._propagate_sender_to_meas(xj_tx, P_j, dt_sender)
        p_j_meas = xj_meas[:3].copy()
        P_j_meas = Pj6_meas[:3, :3]



        '''# -----------------------------
        # sender state at t_meas
        # ORACLE DEBUG VERSION:
        # use true sender position at t_meas from sim memory
        # -----------------------------
        dt_sender = 0.0
        if np.isfinite(t_tx) and np.isfinite(t_meas):
            dt_sender = max(0.0, float(t_meas) - float(t_tx))

        try:
            p_j_true_meas = np.asarray(sim.memory.recall_position(float(t_meas), sender_name), dtype=float).reshape(3)
            p_j_meas = p_j_true_meas.copy()
            sender_oracle_ok = True
        except Exception:
            # fallback to current approximate sender propagation
            xj_tx = np.array([p_j[0], p_j[1], p_j[2], 180.0, 0.2, 0.0], dtype=float)
            xj_meas, _ = self._propagate_sender_to_meas(xj_tx, P_j, dt_sender)
            p_j_meas = xj_meas[:3].copy()
            sender_oracle_ok = False

        # for the oracle test, make sender covariance contribution minimal / disabled
        P_j_meas = np.zeros((3, 3), dtype=float)'''

        # -----------------------------
        # innovation at t_meas
        # -----------------------------
        p_i_meas = x_rx_meas[:3].copy()
        diff = p_i_meas - p_j_meas
        r_hat = float(np.linalg.norm(diff))

        if debug:
            print("\n===== COOP DEBUG (BEFORE UPDATE @ t_meas) =====")
            print(f"t_now={t_now:.3f} | t_meas={t_meas:.3f} | staleness={staleness:.3f}")
            #print(f"sender_oracle_ok={sender_oracle_ok}")
            print(f"Receiver pre-now x={x_pre_now[:3]}")
            print(f"Receiver @t_meas x={x_rx_meas[:3]}")
            print(f"Sender @t_meas x={p_j_meas}")
            print(f"z (range)={z:.3f} | r_hat={r_hat:.3f} | nu={z - r_hat:.3f}")

        if r_hat < self.min_range:
            self._log_coop_reject(
                receiver_name=receiver_name,
                sender_name=sender_name,
                reject_reason="min_range",
                z=z,
                r_hat=np.nan,
                nu=np.nan,
                nis=np.nan,
                S=np.nan,
                R_eff=np.nan,
                packet_age=staleness,
                t_now=t_now,
                t_tx=t_tx,
                t_meas=t_meas,
                dt_sender=dt_sender,
                dt_receiver=t_now - t_meas,
                sender_xyz=tuple(p_j_meas),
                receiver_xyz=tuple(x_rx_meas[:3]),
                receiver_xyz_post=tuple(x_pre_now[:3]),
                traceP_pre=float(np.trace(P_pre_now[:3, :3])),
                traceP_post=float(np.trace(P_pre_now[:3, :3])),
            )
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

        R_sender = self.alpha * float((H_j @ P_j_meas @ H_j.T)[0, 0])
        R_delay = float((self.sigma_rel_speed * staleness) ** 2)
        R_eff = float(self.R_range + R_sender + R_delay + self.R_coop_updates)

        R_eff = max(R_eff, 1e-12)

        '''print('agent n:',receiver_name)
        print('R_eff',R_eff)
        print('R_sender',R_sender)
        print('R_coop_updates',self.R_coop_updates)
        print('R_delay',R_delay)
        print('R_range', self.R_range)'''


        nu = float(z - r_hat)
        self.nu = nu

        S = float((H_i @ P_rx_meas @ H_i.T)[0, 0] + R_eff)
        if S <= 0.0:
            self._log_coop_reject(
                receiver_name=receiver_name,
                sender_name=sender_name,
                reject_reason="nonpositive_S",
                z=z,
                r_hat=r_hat,
                nu=nu,
                nis=np.nan,
                S=S,
                R_eff=R_eff,
                packet_age=staleness,
                t_now=t_now,
                t_tx=t_tx,
                t_meas=t_meas,
                dt_sender=dt_sender,
                dt_receiver=t_now - t_meas,
                sender_xyz=tuple(p_j_meas),
                receiver_xyz=tuple(x_rx_meas[:3]),
                receiver_xyz_post=tuple(x_pre_now[:3]),
                traceP_pre=float(np.trace(P_pre_now[:3, :3])),
                traceP_post=float(np.trace(P_pre_now[:3, :3])),
            )
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

        if debug:
            print("----- AFTER UPDATE @ t_meas -----")
            print(f"x_post_meas={x_post_meas[:3]}")
            print(f"delta_update={(x_post_meas[:3] - x_rx_meas[:3])}")

        nis = float((nu ** 2) / S)
        if nis > self.nis_gate:
            self._log_coop_reject(
                receiver_name=receiver_name,
                sender_name=sender_name,
                reject_reason="nis_gate",
                z=z,
                r_hat=r_hat,
                nu=nu,
                nis=nis,
                S=S,
                R_eff=R_eff,
                packet_age=staleness,
                t_now=t_now,
                t_tx=t_tx,
                t_meas=t_meas,
                dt_sender=dt_sender,
                dt_receiver=t_now - t_meas,
                sender_xyz=tuple(p_j_meas),
                receiver_xyz=tuple(x_rx_meas[:3]),
                receiver_xyz_post=tuple(x_pre_now[:3]),
                traceP_pre=float(np.trace(P_pre_now[:3, :3])),
                traceP_post=float(np.trace(P_pre_now[:3, :3])),
            )
            return

        # -----------------------------
        # replay forward from t_meas to t_now
        # using SAME propagation model as normal prediction
        # -----------------------------
        x_replay = x_post_meas.copy()
        P_replay = P_post_meas.copy()
        model_replay = copy.deepcopy(model_rx_meas)
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
                x_replay, P_replay, model_replay = self._propagate_shadow_mean_cov(
                    receiver_name=receiver_name,
                    x=x_replay,
                    P=P_replay,
                    model_agent=model_replay,
                    dt=dt,
                    sim=sim,
                )
                t_replay = t_k

            x_replay, P_replay = self._apply_local_measurements_to_state(
                x_replay, P_replay,
                meas_depth=float(entry["meas_depth"]),
                meas_heading=float(entry["meas_heading"]),
                meas_body_vel=np.asarray(entry["meas_body_vel"], dtype=float),
            )

        dt_tail = float(t_now - t_replay)
        if dt_tail > 0.0:
            x_replay, P_replay, model_replay = self._propagate_shadow_mean_cov(
                receiver_name=receiver_name,
                x=x_replay,
                P=P_replay,
                model_agent=model_replay,
                dt=dt_tail,
                sim=sim,
            )
            t_replay = float(t_now)

        st.x = x_replay
        st.P = P_replay
        st.t = float(t_now)
        st.model_agent = model_replay
        st.x[3] = self._wrap_deg(st.x[3])
        st.last_coop_update = float(t_now)
        st.quality = float(np.trace(st.P[:3, :3]))

        if debug:
            print("----- AFTER REPLAY TO t_now -----")
            print(f"x_replay={x_replay[:3]}")
            print(f"x_pre_now={x_pre_now[:3]}")
            print(f"delta_replay_vs_pre={(x_replay[:3] - x_pre_now[:3])}")
            print("====================================\n")

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

    def _propagate_shadow(self, agent, sim, model_template, x_state):
        m = copy.deepcopy(model_template)
        dt = float(sim.Dt)

        m.pos = np.array([x_state[0], x_state[1], x_state[2]], dtype=float)
        m.psi = float(x_state[3])
        m.Dt = dt

        if hasattr(m, "incurrent_velocity"):
            m.incurrent_velocity = np.array([x_state[4], x_state[5]], dtype=float)

        if hasattr(m, "last_step_pos"):
            m.last_step_pos = np.array([x_state[0], x_state[1], x_state[2]], dtype=float)

        for attr in (
            "cmd_depth", "cmd_heave", "cmd_heading", "cmd_yawrate",
            "cmd_planar", "cmd_local_vel", "cmd_forces", "other_forces",
            "depth_control", "heading_control", "planar_control"
        ):
            if hasattr(agent, attr):
                setattr(m, attr, copy.deepcopy(getattr(agent, attr)))

        m.measured_depth = float(m.pos[2])
        m.measured_heading = float(m.psi)
        m.measured_pos = m.pos[:2].copy()

        m._update_heading()
        m._update_depth()
        m._update_planar(m.Dt)

        vel_body_nom = self._get_body_velocity_from_shadow(m, x_state, dt)
        vel_ned = self._body_to_ned_2d(vel_body_nom, m.psi)

        x_next = np.array([
            float(x_state[0] + vel_ned[0] * dt),
            float(x_state[1] + vel_ned[1] * dt),
            float(m.pos[2]),
            self._wrap_deg(float(m.psi)),
            float(vel_body_nom[0]),
            float(vel_body_nom[1]),
        ], dtype=float)

        m.pos[0] = x_next[0]
        m.pos[1] = x_next[1]
        m.pos[2] = x_next[2]
        m.psi = x_next[3]

        if hasattr(m, "last_step_pos"):
            m.last_step_pos = m.pos.copy()

        return x_next, m

    def _propagate_shadow_mean_cov(self, receiver_name, x, P, model_agent, dt, sim):
        """
        Propagate mean/covariance with the SAME shadow-model dynamics
        used by the main EKF prediction.
        """
        if dt <= 0.0:
            return np.asarray(x, dtype=float).copy(), np.asarray(P, dtype=float).copy(), copy.deepcopy(model_agent)

        agent = self.agents[receiver_name]

        st_tmp = NavState(
            x=np.asarray(x, dtype=float).copy(),
            P=np.asarray(P, dtype=float).copy(),
            t=0.0,
            model_agent=copy.deepcopy(model_agent),
        )

        # numerical Jacobian using same shadow propagation, but with custom dt
        F = self._numerical_transition_jacobian_with_dt(agent, sim, st_tmp, dt)
        x_next, model_next = self._propagate_shadow_with_dt(agent, sim, st_tmp.model_agent, st_tmp.x, dt)

        Q = np.diag(self.Q_diag * dt)
        P_next = self._symmetrize(F @ st_tmp.P @ F.T + Q)

        return x_next, P_next, model_next

    def _propagate_shadow_with_dt(self, agent, sim, model_template, x_state, dt):
        m = copy.deepcopy(model_template)

        m.pos = np.array([x_state[0], x_state[1], x_state[2]], dtype=float)
        m.psi = float(x_state[3])
        m.Dt = float(dt)

        if hasattr(m, "incurrent_velocity"):
            m.incurrent_velocity = np.array([x_state[4], x_state[5]], dtype=float)

        if hasattr(m, "last_step_pos"):
            m.last_step_pos = np.array([x_state[0], x_state[1], x_state[2]], dtype=float)

        for attr in (
            "cmd_depth", "cmd_heave", "cmd_heading", "cmd_yawrate",
            "cmd_planar", "cmd_local_vel", "cmd_forces", "other_forces",
            "depth_control", "heading_control", "planar_control"
        ):
            if hasattr(agent, attr):
                setattr(m, attr, copy.deepcopy(getattr(agent, attr)))

        m.measured_depth = float(m.pos[2])
        m.measured_heading = float(m.psi)
        m.measured_pos = m.pos[:2].copy()

        m._update_heading()
        m._update_depth()
        m._update_planar(m.Dt)

        vel_body_nom = self._get_body_velocity_from_shadow(m, x_state, dt)
        vel_ned = self._body_to_ned_2d(vel_body_nom, m.psi)

        x_next = np.array([
            float(x_state[0] + vel_ned[0] * dt),
            float(x_state[1] + vel_ned[1] * dt),
            float(m.pos[2]),
            self._wrap_deg(float(m.psi)),
            float(vel_body_nom[0]),
            float(vel_body_nom[1]),
        ], dtype=float)

        m.pos[0] = x_next[0]
        m.pos[1] = x_next[1]
        m.pos[2] = x_next[2]
        m.psi = x_next[3]

        if hasattr(m, "last_step_pos"):
            m.last_step_pos = m.pos.copy()

        return x_next, m

    def _numerical_transition_jacobian(self, agent, sim, st):
        return self._numerical_transition_jacobian_with_dt(agent, sim, st, float(sim.Dt))

    def _numerical_transition_jacobian_with_dt(self, agent, sim, st, dt):
        x0 = st.x.copy()
        n = x0.size
        F = np.zeros((n, n), dtype=float)

        eps = np.array([1e-3, 1e-3, 1e-3, 1e-2, 1e-4, 1e-4], dtype=float)

        for k in range(n):
            dx = np.zeros(n, dtype=float)
            dx[k] = eps[k]

            x_plus, _ = self._propagate_shadow_with_dt(agent, sim, st.model_agent, x0 + dx, dt)
            x_minus, _ = self._propagate_shadow_with_dt(agent, sim, st.model_agent, x0 - dx, dt)

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
            scale=np.sqrt(np.diag(self.sigma_body_vel)),
            size=2
        )
        return vel_body + vel_noise

    def _get_body_velocity_measurement(self, agent, sim):
        vel_body_nom = self._get_body_velocity_from_agent(agent, sim)
        vel_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.sigma_body_vel)),
            size=2
        )
        return vel_body_nom + vel_noise


    def _get_body_velocity_from_agent(self, agent, sim):
        """
        Model-based onboard body-velocity estimate.

        Assumption:
        - the vehicle knows the commanded planar velocity in body frame
        - this is the quantity available onboard from control / dynamic model
        - it does NOT use true global position differences

        For the current simulator setup with hardcoded commanded velocities,
        cmd_local_vel is the most appropriate baseline proxy.
        """
        if hasattr(agent, "cmd_local_vel"):
            vel = np.asarray(agent.cmd_local_vel, dtype=float).reshape(2)
            return vel.copy()

        return np.zeros(2, dtype=float)

    def _get_body_velocity_from_shadow(self, model_agent, x_state, dt):
        if getattr(model_agent, "planar_control", "") == "local_forces":
            if hasattr(model_agent, "incurrent_velocity"):
                return np.asarray(model_agent.incurrent_velocity, dtype=float).reshape(2).copy()

        return np.asarray(x_state[4:6], dtype=float).copy()

    @staticmethod
    def _ned_to_body_2d(v_ned, psi_deg):
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

    def _constvel_jacobian(self, x, dt):
        """
        Retained only for sender covariance propagation in the current simplified setup.
        """
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
        # current simplified scenario: sender mean propagated with known constant motion
        xj_meas = np.asarray(xj_tx, dtype=float).copy()
        vel_ned = self._body_to_ned_2d(xj_meas[4:6], xj_meas[3])
        xj_meas[0] += vel_ned[0] * dt_sender
        xj_meas[1] += vel_ned[1] * dt_sender
        xj_meas[3] = self._wrap_deg(xj_meas[3])

        Fj = self._constvel_jacobian(xj_tx, dt_sender)
        Pj6 = np.zeros((6, 6), dtype=float)
        Pj6[:3, :3] = P_j
        Qj = np.diag(self.Q_diag * max(dt_sender, 0.0))
        Pj6_meas = Fj @ Pj6 @ Fj.T + Qj
        Pj6_meas = self._symmetrize(Pj6_meas)
        return xj_meas, Pj6_meas

    # ==========================================================
    # History helpers
    # ==========================================================

    def _append_history_snapshot(self, st, t, x, P, model_agent, meas_depth, meas_heading, meas_body_vel):
        st.history.append({
            "kind": "snapshot",
            "t": float(t),
            "x": np.asarray(x, dtype=float).copy(),
            "P": np.asarray(P, dtype=float).copy(),
            "model_agent": copy.deepcopy(model_agent),
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
    # Logging helpers
    # ==========================================================

    def _log_coop_reject(
        self,
        receiver_name,
        sender_name,
        reject_reason,
        z,
        r_hat,
        nu,
        nis,
        S,
        R_eff,
        packet_age,
        t_now,
        t_tx,
        t_meas,
        dt_sender,
        dt_receiver,
        sender_xyz,
        receiver_xyz,
        receiver_xyz_post,
        traceP_pre,
        traceP_post,
    ):
        self.coop_update_log.append({
            "t": float(t_now),
            "receiver": receiver_name,
            "sender": sender_name,
            "accepted": 0,
            "reject_reason": str(reject_reason),
            "range": float(z),
            "r_hat": float(r_hat),
            "nu": float(nu),
            "nis": float(nis),
            "S": float(S),
            "R_eff": float(R_eff),
            "packet_age": float(packet_age),
            "t_tx_payload": float(t_tx),
            "t_meas": float(t_meas),
            "dt_sender": float(dt_sender),
            "dt_receiver": float(dt_receiver),
            "sender_x_meas": float(sender_xyz[0]),
            "sender_y_meas": float(sender_xyz[1]),
            "sender_z_meas": float(sender_xyz[2]),
            "receiver_x_meas": float(receiver_xyz[0]),
            "receiver_y_meas": float(receiver_xyz[1]),
            "receiver_z_meas": float(receiver_xyz[2]),
            "receiver_x_post": float(receiver_xyz_post[0]),
            "receiver_y_post": float(receiver_xyz_post[1]),
            "receiver_z_post": float(receiver_xyz_post[2]),
            "traceP_pre": float(traceP_pre),
            "traceP_post": float(traceP_post),
        })

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
                "nu": float(self.nu)
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