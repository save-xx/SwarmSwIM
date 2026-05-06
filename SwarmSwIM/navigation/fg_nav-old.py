import numpy as np
import casadi
from dataclasses import dataclass, field

from .base_nav import BaseNavFilter


@dataclass
class NavState:
    """
    Navigation state container for one agent.

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

    # FG-specific buffers
    fg_range_window: list = field(default_factory=list)
    fg_gps_window: list = field(default_factory=list)

    fg_last_solution: dict = field(default_factory=dict)


class FGNavFilter(BaseNavFilter):
    """
    Receiver-local sliding-window factor graph navigation filter.

    Current graph
    -------------
    Per agent, over a recent time horizon:
    - one 2D node [x, y] per unique event time
    - prior on oldest node
    - DR continuity factors between consecutive nodes
    - range factors at range-event times
    - GPS XY factors at GPS-event times

    Trigger
    -------
    Solve when:
    - a new valid cooperative range event is received
    - a new GPS fix is added
    """

    def __init__(
        self,
        Q_diag=(0.01, 0.01, 0.0001, 0.03, 0.01, 0.02),
        P0_diag=(0.25, 0.25, 1e-4, 9.0, 0.01, 0.01),
        R_depth=1e-6,
        R_heading_deg=4.0,
        R_body_vel_diag=np.array([0.003, 0.003]),

        surface_agents=("A04", "A02"),
        var_gps_fix=np.array([0.25, 0.25]),
        R_surface_pos_diag=(0.04, 0.04),

        history_length=1000,
        writeback=True,
        keep_history=True,
        rng_seed=50,

        fg_time_horizon=30.0,
        max_range_events=10,
        max_gps_events=10,
        min_fg_nodes=2,

        fg_sigma_prior=0.5,
        fg_sigma_dr=0.5,
        fg_sigma_range=0.3,
        fg_sigma_gps=0.2,
        fg_robust_delta=2.0,
        fg_use_robust=True,
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.P0_diag = np.asarray(P0_diag, dtype=float)

        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))

        self.var_gps_fix = np.diag(np.asarray(var_gps_fix, dtype=float))
        self.R_surface_pos = np.diag(np.asarray(R_surface_pos_diag, dtype=float))

        self.history_length = int(history_length)
        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.surface_agents = set(surface_agents)

        self.fg_time_horizon = float(fg_time_horizon)
        self.max_range_events = int(max_range_events)
        self.max_gps_events = int(max_gps_events)
        self.min_fg_nodes = int(min_fg_nodes)

        self.fg_sigma_prior = float(fg_sigma_prior)
        self.fg_sigma_dr = float(fg_sigma_dr)
        self.fg_sigma_range = float(fg_sigma_range)
        self.fg_sigma_gps = float(fg_sigma_gps)
        self.fg_robust_delta = float(fg_robust_delta)
        self.fg_use_robust = bool(fg_use_robust)

        self.rng = np.random.default_rng(rng_seed)

        self.coop_update_log = []
        self.fg_last_cost = np.nan

    # ==========================================================
    # Base hooks
    # ==========================================================

    def _init_filter(self, agent):
        vel0 = self._get_body_velocity_measurement(agent, None)

        gps_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2
        )

        x0 = np.array([
            float(agent.pos[0]) + gps_noise[0],
            float(agent.pos[1]) + gps_noise[1],
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

        # GPS handled in FG
        '''gps_added = self._maybe_add_gps_event(agent, sim)
        if gps_added:
            result = self._solve_agent_fg(st, float(sim.time))
            self._apply_fg_solution(st, result, sim)'''

    def update_surface_position(self, agent, sim):
        
        if not self._is_surface_agent(agent):
            return

        st = self.filters[agent.name]

        pos_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2
        )

        z_xy = np.array([
            float(agent.pos[0]) + pos_noise[0],
            float(agent.pos[1]) + pos_noise[1],
        ], dtype=float)

        H_xy = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        ], dtype=float)

        '''self._ekf_update_linear(
            st,
            z=z_xy,
            h=st.x[0:2].copy(),
            H=H_xy,
            R=self.R_surface_pos,
            angle_idx=None,
        )'''

        gps_added = self._maybe_add_gps_event(agent, sim)
        if gps_added:
            result = self._solve_agent_fg(st, float(sim.time))
            self._apply_fg_solution(st, result, sim)

        st.quality = float(np.trace(st.P[:3, :3]))

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

            payload = msg.payload
            if payload is None or payload.get("type") != "nav":
                continue

            st = self.filters[receiver_name]
            meas = receiver.AcousticRange[sender_name]

            t_meas = meas.get("t_meas", None)
            if t_meas is None:
                continue
            t_meas = float(t_meas)

            last_t_meas = st.last_coop_meas_time.get(sender_name, -np.inf)
            if t_meas <= last_t_meas:
                continue

            event = self._build_range_event(
                receiver=receiver,
                sender_name=sender_name,
                payload=payload,
                meas=meas,
                st=st,
                sim=sim,
            )
            if event is None:
                continue

            self._append_range_event(st, event)

            result = self._solve_agent_fg(st, float(sim.time))
            self._apply_fg_solution(st, result, sim)

            st.last_coop_meas_time[sender_name] = t_meas

            self.coop_update_log.append({
                "t": float(sim.time),
                "receiver": receiver_name,
                "sender": sender_name,
                "accepted": int(result is not None and result.get("success", False)),
                "range": float(event["z_range"]),
                "t_tx_payload": float(event["t_tx"]),
                "t_meas": float(event["t_meas"]),
                "n_range": int(len(st.fg_range_window)),
                "n_gps": int(len(st.fg_gps_window)),
                "receiver_x_post": float(st.x[0]),
                "receiver_y_post": float(st.x[1]),
                "traceP_post": float(np.trace(st.P[:3, :3])),
                "fg_cost": float(self.fg_last_cost) if np.isfinite(self.fg_last_cost) else np.nan,
            })

        if self.writeback:
            self._writeback_all(sim)

    # ==========================================================
    # FG event management
    # ==========================================================

    def _maybe_add_gps_event(self, agent, sim):
        if not self._is_surface_agent(agent):
            return False

        st = self.filters[agent.name]

        gps_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2
        )

        z_xy = np.array([
            float(agent.pos[0]) + gps_noise[0],
            float(agent.pos[1]) + gps_noise[1],
        ], dtype=float)

        if not np.all(np.isfinite(z_xy)):
            return False

        event = {
            "type": "gps",
            "t_meas": float(sim.time),
            "gps_xy": z_xy.copy(),
            "receiver_dr_xy": st.x[:2].copy(),
        }

        # avoid duplicates at same timestamp
        if st.fg_gps_window:
            if abs(float(st.fg_gps_window[-1]["t_meas"]) - float(sim.time)) < 1e-12:
                return False

        self._append_gps_event(st, event)
        return True

    def _build_range_event(self, receiver, sender_name, payload, meas, st, sim):
        if payload is None:
            return None
        if payload.get("type") != "nav":
            return None
        if "pos" not in payload or "cov" not in payload:
            return None
        if "range" not in meas:
            return None

        t_meas = meas.get("t_meas", None)
        if t_meas is None:
            return None
        t_meas = float(t_meas)

        t_tx = float(payload.get("tx_time", meas.get("t_tx", sim.time)))
        t_now = float(sim.time)

        z_range = float(meas["range"])
        if not np.isfinite(z_range):
            return None

        p_j = np.asarray(payload["pos"], dtype=float).reshape(-1)
        if p_j.size != 3 or not np.all(np.isfinite(p_j)):
            return None

        psi_j = float(payload.get("heading", np.nan))
        if not np.isfinite(psi_j):
            psi_j = 0.0

        v_j = np.asarray(payload.get("body_vel", [0.0, 0.0]), dtype=float).reshape(-1)
        if v_j.size != 2 or not np.all(np.isfinite(v_j)):
            v_j = np.zeros(2, dtype=float)

        P_j = self._parse_sender_cov(payload.get("cov", None))
        if P_j is None:
            P_j = np.eye(3, dtype=float)

        z_i = float(st.x[2])
        if not np.isfinite(z_i):
            return None

        event = {
            "type": "range",
            "t_meas": t_meas,
            "t_tx": t_tx,
            "t_now": t_now,
            "sender": sender_name,
            "z_range": z_range,
            "sender_pos": p_j.copy(),
            "sender_heading": psi_j,
            "sender_body_vel": v_j.copy(),
            "sender_cov": P_j.copy(),
            "receiver_depth": z_i,
            "receiver_dr_xy": st.x[:2].copy(),
        }
        return event

    def _append_range_event(self, st, event):
        st.fg_range_window.append(event)
        if len(st.fg_range_window) > self.max_range_events:
            st.fg_range_window[:] = st.fg_range_window[-self.max_range_events:]

    def _append_gps_event(self, st, event):
        st.fg_gps_window.append(event)
        if len(st.fg_gps_window) > self.max_gps_events:
            st.fg_gps_window[:] = st.fg_gps_window[-self.max_gps_events:]

    def _select_recent_events(self, st, t_now):
        t_min = float(t_now) - self.fg_time_horizon

        range_events = [
            ev for ev in st.fg_range_window
            if float(ev["t_meas"]) >= t_min
        ]
        gps_events = [
            ev for ev in st.fg_gps_window
            if float(ev["t_meas"]) >= t_min
        ]

        if len(range_events) > self.max_range_events:
            range_events = range_events[-self.max_range_events:]
        if len(gps_events) > self.max_gps_events:
            gps_events = gps_events[-self.max_gps_events:]

        return range_events, gps_events

    def _build_fg_nodes_and_maps(self, range_events, gps_events):
        """
        Returns:
        - node_times: sorted unique timestamps
        - time_to_idx: dict
        - range_by_idx: dict[idx] -> list of range events
        - gps_by_idx: dict[idx] -> list of gps events
        """
        all_times = []
        for ev in range_events:
            all_times.append(float(ev["t_meas"]))
        for ev in gps_events:
            all_times.append(float(ev["t_meas"]))

        node_times = sorted(set(all_times))
        time_to_idx = {t: k for k, t in enumerate(node_times)}

        range_by_idx = {k: [] for k in range(len(node_times))}
        gps_by_idx = {k: [] for k in range(len(node_times))}

        for ev in range_events:
            range_by_idx[time_to_idx[float(ev["t_meas"])]] .append(ev)
        for ev in gps_events:
            gps_by_idx[time_to_idx[float(ev["t_meas"])]] .append(ev)

        return node_times, time_to_idx, range_by_idx, gps_by_idx

    def _reference_xy_at_time(self, st, t_query):
        """
        Simple reference extraction from history.
        Fallback: current state.
        """
        best = None
        best_dt = np.inf

        for entry in reversed(st.history):
            t_entry = float(entry.get("t", np.inf))
            dt = abs(t_entry - float(t_query))
            if dt < best_dt:
                best_dt = dt
                best = entry
            if dt < 1e-9:
                break

        if best is not None and "x" in best:
            x = np.asarray(best["x"], dtype=float).reshape(-1)
            if x.size >= 2 and np.all(np.isfinite(x[:2])):
                return x[:2].copy()

        return st.x[:2].copy()

    # ==========================================================
    # FG solve
    # ==========================================================

    def _solve_agent_fg(self, st, t_now):
        range_events, gps_events = self._select_recent_events(st, t_now)

        node_times, _, range_by_idx, gps_by_idx = self._build_fg_nodes_and_maps(
            range_events, gps_events
        )

        N = len(node_times)
        if N < self.min_fg_nodes:
            return None

        X = casadi.SX.sym("X_fg", 2 * N)

        def xy_k(k):
            return X[2 * k: 2 * k + 2]

        def pseudo_huber(z, delta):
            return 2.0 * delta * delta * (casadi.sqrt(1.0 + (z / delta) ** 2) - 1.0)

        J = casadi.SX(0)

        # reference XY for each node time
        ref_xy = [self._reference_xy_at_time(st, t) for t in node_times]

        # prior on oldest node
        r0 = (xy_k(0) - ref_xy[0]) / float(self.fg_sigma_prior)
        J += casadi.dot(r0, r0)

        # DR continuity between consecutive nodes
        for k in range(1, N):
            delta_dr = ref_xy[k] - ref_xy[k - 1]
            r_dr = (xy_k(k) - xy_k(k - 1) - delta_dr) / float(self.fg_sigma_dr)
            J += casadi.dot(r_dr, r_dr)

        # range factors
        for k in range(N):
            for ev in range_by_idx[k]:
                p_j = np.asarray(ev["sender_pos"], dtype=float).reshape(3)
                z_i = float(ev["receiver_depth"])
                z_j = float(p_j[2])
                z_range = float(ev["z_range"])

                if not np.isfinite(z_i) or not np.isfinite(z_j) or not np.isfinite(z_range):
                    continue

                dx = xy_k(k)[0] - float(p_j[0])
                dy = xy_k(k)[1] - float(p_j[1])
                dz = float(z_i - z_j)

                r_hat = casadi.sqrt(dx * dx + dy * dy + dz * dz)
                r = (r_hat - z_range) / float(self.fg_sigma_range)

                if self.fg_use_robust:
                    J += pseudo_huber(r, float(self.fg_robust_delta))
                else:
                    J += r * r

        # GPS factors
        for k in range(N):
            for ev in gps_by_idx[k]:
                z_xy = np.asarray(ev["gps_xy"], dtype=float).reshape(2)
                if not np.all(np.isfinite(z_xy)):
                    continue

                r_gps = (xy_k(k) - z_xy) / float(self.fg_sigma_gps)
                J += casadi.dot(r_gps, r_gps)

        x0 = []
        for p_ref in ref_xy:
            x0.extend([float(p_ref[0]), float(p_ref[1])])

        nlp = {"x": X, "f": J}

        try:
            solver = casadi.nlpsol(
                f"fg_solver_{id(st)}_{N}",
                "ipopt",
                nlp,
                {
                    "print_time": 0,
                    "ipopt.print_level": 0,
                }
            )
            sol = solver(x0=x0)
            x_opt = np.asarray(sol["x"].full()).reshape(-1)
            cost = float(sol["f"])
        except Exception:
            return {
                "success": False,
                "xy_last": None,
                "xy_all": None,
                "cost": np.inf,
                "node_times": node_times,
            }

        xy_all = []
        for k in range(N):
            xy_all.append(x_opt[2 * k: 2 * k + 2].copy())

        return {
            "success": True,
            "xy_last": xy_all[-1].copy(),
            "xy_all": xy_all,
            "cost": cost,
            "node_times": node_times,
        }

    def _apply_fg_solution(self, st, result, sim):
        if result is None or not result.get("success", False):
            return

        st.x[0:2] = np.asarray(result["xy_last"], dtype=float).reshape(2)
        st.t = float(sim.time)
        st.last_coop_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))
        self.fg_last_cost = float(result.get("cost", np.nan))

    # ==========================================================
    # Transition model
    # ==========================================================

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
        return self._get_body_velocity_from_agent(agent, sim)

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
    # EKF-style local helpers
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
                "fg_cost": float(self.fg_last_cost) if np.isfinite(self.fg_last_cost) else np.nan,
                "n_fg_range": int(len(st.fg_range_window)),
                "n_fg_gps": int(len(st.fg_gps_window)),
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