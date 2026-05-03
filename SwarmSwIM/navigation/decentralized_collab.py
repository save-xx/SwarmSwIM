from dataclasses import dataclass, field

try:
    import casadi as ca
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "DecentralizedCollabNavFilter requires CasADi. Install project "
        "dependencies with `python3 -m pip install -e .` or install it "
        "directly with `python3 -m pip install casadi`."
    ) from exc
import numpy as np

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

    collab_range_window: list = field(default_factory=list)
    collab_gps_window: list = field(default_factory=list)
    collab_last_solution: dict = field(default_factory=dict)
    collab_last_cost: float = np.nan
    collab_last_status: str = ""
    next_gps_update: float = 0.0


class DecentralizedCollaborativeNavFilter(BaseNavFilter):
    """
    Online decentralized sliding-window collaborative optimizer.

    This is an online adaptation of the offline CasADi optimizer in the shared
    script. Each receiving agent solves its own local XY trajectory window while
    treating neighbor states received in acoustic NAV packets as fixed inputs.

    Objective terms:
    - dead-reckoning / velocity continuity
    - acoustic range
    - Doppler range-rate
    - soft anchor and optional surface GPS factors

    Hard constraints:
    - minimum receiver-to-sender planar distance at range-event nodes
    - maximum planar speed between consecutive window nodes
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

        history_length=2000,
        writeback=True,
        keep_history=True,
        rng_seed=50,

        window_duration=60.0,
        max_range_events=None,
        max_gps_events=None,
        min_nodes=2,

        sigma_v=0.1,
        sigma_r=0.32,
        sigma_anchor=0.01,
        sigma_doppler=0.1,
        sigma_gps=None,
        use_doppler=True,

        min_distance=0.2,
        max_speed=3.0,

        gps_update_period=1.0,
        solve_on_gps=True,

        payload_key=None,
        ipopt_print_level=0,
        solver_options=None,
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.P0_diag = np.asarray(P0_diag, dtype=float)

        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))

        self.var_gps_fix = np.diag(np.asarray(var_gps_fix, dtype=float))
        self.R_surface_pos = np.diag(np.asarray(R_surface_pos_diag, dtype=float))

        self.surface_agents = set(surface_agents)

        self.history_length = int(history_length)
        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.window_duration = float(window_duration)
        self.max_range_events = None if max_range_events is None else int(max_range_events)
        self.max_gps_events = None if max_gps_events is None else int(max_gps_events)
        self.min_nodes = int(min_nodes)

        self.sigma_v = float(sigma_v)
        self.sigma_r = float(sigma_r)
        self.sigma_anchor = float(sigma_anchor)
        self.sigma_doppler = float(sigma_doppler)
        self.sigma_gps = (
            float(np.sqrt(np.mean(np.diag(self.R_surface_pos))))
            if sigma_gps is None else float(sigma_gps)
        )
        self.use_doppler = bool(use_doppler)

        self.min_distance = float(min_distance)
        self.max_speed = float(max_speed)

        self.gps_update_period = float(gps_update_period)
        self.solve_on_gps = bool(solve_on_gps)

        self.payload_key = payload_key
        self.ipopt_print_level = int(ipopt_print_level)
        self.solver_options = dict(solver_options or {})

        self.rng = np.random.default_rng(rng_seed)

        self.coop_update_log = []
        self.collab_last_cost = np.nan
        self.collab_last_status = ""
        self._solver_counter = 0

    # ==========================================================
    # Base hooks
    # ==========================================================

    def _init_filter(self, agent):
        vel0 = self._get_body_velocity_measurement(agent, None)

        gps_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2,
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
            next_gps_update=0.0,
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

        if self.solve_on_gps and self._maybe_add_gps_event(agent, sim):
            result = self._solve_agent_window(st, float(sim.time))
            self._apply_window_solution(agent.name, st, result, sim)

    def update_surface_position(self, agent, sim):
        """
        Direct EKF-style surface XY update retained for debugging.

        The online collaborative optimizer normally handles surface position
        as GPS factors in the sliding window instead.
        """
        if not self._is_surface_agent(agent):
            return

        st = self.filters[agent.name]

        pos_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2,
        )

        z_xy = np.array([
            float(agent.pos[0]) + pos_noise[0],
            float(agent.pos[1]) + pos_noise[1],
        ], dtype=float)

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

            payload = self._extract_nav_payload(msg.payload)
            if payload is None:
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
                receiver_name=receiver_name,
                sender_name=sender_name,
                payload=payload,
                meas=meas,
                st=st,
                sim=sim,
            )
            if event is None:
                continue

            self._append_range_event(st, event)
            result = self._solve_agent_window(st, float(sim.time))
            accepted = self._apply_window_solution(receiver_name, st, result, sim)

            st.last_coop_meas_time[sender_name] = t_meas

            self.coop_update_log.append({
                "t": float(sim.time),
                "receiver": receiver_name,
                "sender": sender_name,
                "accepted": int(accepted),
                "range": float(event["z_range"]),
                "doppler": float(event["z_doppler"]),
                "t_tx_payload": float(event["t_tx"]),
                "t_meas": float(event["t_meas"]),
                "n_range": int(len(st.collab_range_window)),
                "n_gps": int(len(st.collab_gps_window)),
                "n_nodes": int(result.get("n_nodes", 0)) if result else 0,
                "collab_cost": float(result.get("cost", np.nan)) if result else np.nan,
                "solver_status": str(result.get("status", "")) if result else "",
                "receiver_x_post": float(st.x[0]),
                "receiver_y_post": float(st.x[1]),
                "receiver_z_post": float(st.x[2]),
                "traceP_post": float(np.trace(st.P[:3, :3])),
            })

        if self.writeback:
            self._writeback_all(sim)

    # ==========================================================
    # Event management
    # ==========================================================

    def _extract_nav_payload(self, payload):
        if payload is None:
            return None

        if self.payload_key is None:
            if payload.get("type") != "nav":
                return None
            return payload

        nested = payload.get(self.payload_key, None)
        if nested is None:
            return None
        if nested.get("type", "nav") != "nav":
            return None
        return nested

    def _maybe_add_gps_event(self, agent, sim):
        if not self._is_surface_agent(agent):
            return False

        st = self.filters[agent.name]
        t_now = float(sim.time)

        if self.gps_update_period > 0.0:
            if t_now + 1e-12 < float(st.next_gps_update):
                return False
            st.next_gps_update = t_now + self.gps_update_period

        pos_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2,
        )

        z_xy = np.array([
            float(agent.pos[0]) + pos_noise[0],
            float(agent.pos[1]) + pos_noise[1],
        ], dtype=float)

        if not np.all(np.isfinite(z_xy)):
            return False

        if st.collab_gps_window:
            t_last = float(st.collab_gps_window[-1]["t_meas"])
            if abs(t_last - t_now) < 1e-12:
                return False

        self._append_gps_event(st, {
            "type": "gps",
            "t_meas": t_now,
            "gps_xy": z_xy.copy(),
        })
        return True

    def _build_range_event(self, receiver_name, sender_name, payload, meas, st, sim):
        if "pos" not in payload:
            return None
        if "range" not in meas:
            return None

        t_meas = meas.get("t_meas", None)
        if t_meas is None:
            return None
        t_meas = float(t_meas)

        z_range = float(meas["range"])
        if not np.isfinite(z_range) or z_range <= 0.0:
            return None

        z_doppler = float(meas.get("doppler", np.nan))

        p_j_tx = np.asarray(payload["pos"], dtype=float).reshape(-1)
        if p_j_tx.size != 3 or not np.all(np.isfinite(p_j_tx)):
            return None

        psi_j = float(payload.get("heading", np.nan))
        if not np.isfinite(psi_j):
            psi_j = 0.0

        v_j_body = np.asarray(payload.get("body_vel", [0.0, 0.0]), dtype=float).reshape(-1)
        if v_j_body.size != 2 or not np.all(np.isfinite(v_j_body)):
            v_j_body = np.zeros(2, dtype=float)

        t_tx = float(payload.get("tx_time", meas.get("t_tx", sim.time)))
        dt_sender = max(0.0, t_meas - t_tx)

        sender_vel_xy = self._body_to_ned_2d(v_j_body, psi_j)
        p_j_meas = p_j_tx.copy()
        p_j_meas[0:2] = p_j_meas[0:2] + sender_vel_xy * dt_sender

        receiver_state = self._reference_state_at_time(st, t_meas)
        if receiver_state is None:
            receiver_depth = float(st.x[2])
        else:
            receiver_depth = float(receiver_state["x"][2])

        if not np.isfinite(receiver_depth):
            return None

        P_j = self._parse_sender_cov(payload.get("cov", None))
        if P_j is None:
            P_j = np.eye(3, dtype=float)

        return {
            "type": "range",
            "receiver": receiver_name,
            "sender": sender_name,
            "t_meas": t_meas,
            "t_tx": t_tx,
            "z_range": z_range,
            "z_doppler": z_doppler,
            "sender_pos_tx": p_j_tx.copy(),
            "sender_pos_meas": p_j_meas.copy(),
            "sender_heading": psi_j,
            "sender_body_vel": v_j_body.copy(),
            "sender_vel_xy": sender_vel_xy.copy(),
            "sender_cov": P_j.copy(),
            "receiver_depth": receiver_depth,
        }

    def _append_range_event(self, st, event):
        st.collab_range_window.append(event)
        if self.max_range_events is not None and len(st.collab_range_window) > self.max_range_events:
            st.collab_range_window[:] = st.collab_range_window[-self.max_range_events:]

    def _append_gps_event(self, st, event):
        st.collab_gps_window.append(event)
        if self.max_gps_events is not None and len(st.collab_gps_window) > self.max_gps_events:
            st.collab_gps_window[:] = st.collab_gps_window[-self.max_gps_events:]

    def _select_recent_events(self, st, t_now):
        t_min = float(t_now) - self.window_duration

        range_events = [
            ev for ev in st.collab_range_window
            if t_min - 1e-12 <= float(ev["t_meas"]) <= float(t_now) + 1e-12
        ]
        gps_events = [
            ev for ev in st.collab_gps_window
            if t_min - 1e-12 <= float(ev["t_meas"]) <= float(t_now) + 1e-12
        ]

        if self.max_range_events is not None and len(range_events) > self.max_range_events:
            range_events = range_events[-self.max_range_events:]
        if self.max_gps_events is not None and len(gps_events) > self.max_gps_events:
            gps_events = gps_events[-self.max_gps_events:]

        return range_events, gps_events

    # ==========================================================
    # Sliding-window solve
    # ==========================================================

    def _solve_agent_window(self, st, t_now):
        range_events, gps_events = self._select_recent_events(st, t_now)
        if not range_events and not gps_events:
            return None

        node_times = self._build_node_times(st, float(t_now), range_events, gps_events)
        N = len(node_times)
        if N < self.min_nodes:
            return None

        range_by_idx = {k: [] for k in range(N)}
        gps_by_idx = {k: [] for k in range(N)}

        for ev in range_events:
            range_by_idx[self._nearest_time_index(node_times, float(ev["t_meas"]))].append(ev)
        for ev in gps_events:
            gps_by_idx[self._nearest_time_index(node_times, float(ev["t_meas"]))].append(ev)

        X = ca.SX.sym("X_dc", 2 * N)

        def xy_k(k):
            return X[2 * k: 2 * k + 2]

        J = ca.SX(0)
        g_expr = []
        lbg = []
        ubg = []

        ref_xy = [self._reference_xy_at_time(st, t) for t in node_times]

        r_anchor = (xy_k(0) - ref_xy[0]) / max(self.sigma_anchor, 1e-12)
        J += ca.dot(r_anchor, r_anchor)

        for k in range(N - 1):
            t0 = node_times[k]
            t1 = node_times[k + 1]
            dt = float(t1 - t0)
            if dt <= 1e-9:
                continue

            v0 = self._inertial_velocity_at_time(st, t0)
            v1 = self._inertial_velocity_at_time(st, t1)
            v_avg = 0.5 * (v0 + v1)

            dx_pred = float(v_avg[0]) * dt
            dy_pred = float(v_avg[1]) * dt

            r_dx = ((xy_k(k + 1)[0] - xy_k(k)[0]) - dx_pred) / max(self.sigma_v, 1e-12)
            r_dy = ((xy_k(k + 1)[1] - xy_k(k)[1]) - dy_pred) / max(self.sigma_v, 1e-12)
            J += r_dx * r_dx + r_dy * r_dy

            step_dx = xy_k(k + 1)[0] - xy_k(k)[0]
            step_dy = xy_k(k + 1)[1] - xy_k(k)[1]
            step_dist = ca.sqrt(step_dx * step_dx + step_dy * step_dy + 1e-9)
            g_expr.append(step_dist - self.max_speed * dt)
            lbg.append(-ca.inf)
            ubg.append(0.0)

        for k in range(N):
            for ev in range_by_idx[k]:
                sender_pos = np.asarray(ev["sender_pos_meas"], dtype=float).reshape(3)
                sender_xy = sender_pos[:2]
                sender_z = float(sender_pos[2])
                receiver_z = float(ev["receiver_depth"])

                dx = float(sender_xy[0]) - xy_k(k)[0]
                dy = float(sender_xy[1]) - xy_k(k)[1]
                dz = float(sender_z - receiver_z)

                dist_planar = ca.sqrt(dx * dx + dy * dy + 1e-9)
                dist_3d = ca.sqrt(dx * dx + dy * dy + dz * dz + 1e-9)

                r_range = (float(ev["z_range"]) - dist_3d) / max(self.sigma_r, 1e-12)
                J += r_range * r_range

                if self.use_doppler and np.isfinite(float(ev["z_doppler"])):
                    v_rx = self._inertial_velocity_at_time(st, float(ev["t_meas"]))
                    v_tx = np.asarray(ev["sender_vel_xy"], dtype=float).reshape(2)
                    los_x = dx / dist_planar
                    los_y = dy / dist_planar
                    dop_pred = (float(v_tx[0] - v_rx[0]) * los_x
                                + float(v_tx[1] - v_rx[1]) * los_y)
                    r_dop = (dop_pred - float(ev["z_doppler"])) / max(self.sigma_doppler, 1e-12)
                    J += r_dop * r_dop

                g_expr.append(dist_planar - self.min_distance)
                lbg.append(0.0)
                ubg.append(ca.inf)

            for ev in gps_by_idx[k]:
                z_xy = np.asarray(ev["gps_xy"], dtype=float).reshape(2)
                if not np.all(np.isfinite(z_xy)):
                    continue
                r_gps = (xy_k(k) - z_xy) / max(self.sigma_gps, 1e-12)
                J += ca.dot(r_gps, r_gps)

        x0 = self._build_initial_guess(st, node_times, ref_xy)

        if g_expr:
            g = ca.vertcat(*g_expr)
        else:
            g = ca.SX.zeros(0, 1)

        nlp = {"x": X, "f": J, "g": g}

        opts = {
            "print_time": 0,
            "ipopt.print_level": self.ipopt_print_level,
            "ipopt.sb": "yes",
        }
        opts.update(self.solver_options)

        self._solver_counter += 1
        solver_name = f"dc_collab_{id(st)}_{self._solver_counter}"

        try:
            solver = ca.nlpsol(solver_name, "ipopt", nlp, opts)
            sol = solver(x0=x0, lbg=lbg, ubg=ubg)
            stats = solver.stats()
            success = bool(stats.get("success", False))
            status = str(stats.get("return_status", ""))
            x_opt = np.asarray(sol["x"].full()).reshape(-1)
            cost = float(sol["f"])
        except Exception as exc:
            return {
                "success": False,
                "status": str(exc),
                "cost": np.inf,
                "n_nodes": N,
                "node_times": node_times,
                "xy_all": None,
                "xy_now": None,
            }

        xy_all = []
        for k in range(N):
            xy_all.append(x_opt[2 * k: 2 * k + 2].copy())

        return {
            "success": success,
            "status": status,
            "cost": cost,
            "n_nodes": N,
            "node_times": node_times,
            "xy_all": xy_all,
            "xy_now": xy_all[-1].copy(),
        }

    def _apply_window_solution(self, agent_name, st, result, sim):
        if result is None or not result.get("success", False):
            return False

        st.x[0:2] = np.asarray(result["xy_now"], dtype=float).reshape(2)
        st.t = float(sim.time)
        st.last_coop_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))

        st.collab_last_solution = {
            "node_times": [float(t) for t in result["node_times"]],
            "xy_all": [np.asarray(xy, dtype=float).copy() for xy in result["xy_all"]],
        }

        self.collab_last_cost = float(result.get("cost", np.nan))
        self.collab_last_status = str(result.get("status", ""))
        st.collab_last_cost = self.collab_last_cost
        st.collab_last_status = self.collab_last_status

        return True

    def _build_node_times(self, st, t_now, range_events, gps_events):
        history_bounds = self._history_time_bounds(st)
        history_start = history_bounds[0] if history_bounds is not None else float(t_now)
        anchor_time = max(float(t_now) - self.window_duration, history_start)

        times = [anchor_time, float(t_now)]
        for ev in range_events:
            times.append(float(ev["t_meas"]))
        for ev in gps_events:
            times.append(float(ev["t_meas"]))

        return self._unique_sorted_times(times)

    @staticmethod
    def _unique_sorted_times(times, tol=1e-9):
        out = []
        for t in sorted(float(v) for v in times if np.isfinite(float(v))):
            if not out or abs(t - out[-1]) > tol:
                out.append(t)
        return out

    @staticmethod
    def _nearest_time_index(node_times, t_query):
        arr = np.asarray(node_times, dtype=float)
        return int(np.argmin(np.abs(arr - float(t_query))))

    def _build_initial_guess(self, st, node_times, ref_xy):
        x0 = []
        prev = st.collab_last_solution if st.collab_last_solution else None

        for k, t in enumerate(node_times):
            xy = None
            if prev:
                xy = self._interpolate_solution(prev, float(t))
            if xy is None:
                xy = np.asarray(ref_xy[k], dtype=float).reshape(2)
            x0.extend([float(xy[0]), float(xy[1])])

        return np.asarray(x0, dtype=float)

    @staticmethod
    def _interpolate_solution(solution, t_query):
        times = np.asarray(solution.get("node_times", []), dtype=float)
        xy_all = solution.get("xy_all", [])
        if times.size == 0 or len(xy_all) != times.size:
            return None

        xy_arr = np.asarray(xy_all, dtype=float).reshape(times.size, 2)
        t_query = float(t_query)

        if t_query <= times[0]:
            return xy_arr[0].copy()
        if t_query >= times[-1]:
            return xy_arr[-1].copy()

        k_hi = int(np.searchsorted(times, t_query, side="right"))
        k_lo = k_hi - 1
        dt = times[k_hi] - times[k_lo]
        if dt <= 1e-12:
            return xy_arr[k_lo].copy()

        alpha = (t_query - times[k_lo]) / dt
        return xy_arr[k_lo] + alpha * (xy_arr[k_hi] - xy_arr[k_lo])

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
    # Local measurement helpers
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
            angle_idx=None,
        )

        H_psi = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]], dtype=float)
        self._ekf_update_linear(
            st,
            z=np.array([float(z_psi)], dtype=float),
            h=np.array([st.x[3]], dtype=float),
            H=H_psi,
            R=np.array([[self.R_heading]], dtype=float),
            angle_idx=0,
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
            angle_idx=None,
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

    def _history_snapshots(self, st):
        return [entry for entry in st.history if entry.get("kind") == "snapshot"]

    def _history_time_bounds(self, st):
        snapshots = self._history_snapshots(st)
        if not snapshots:
            return None
        return float(snapshots[0]["t"]), float(snapshots[-1]["t"])

    def _reference_state_at_time(self, st, t_query):
        snapshots = self._history_snapshots(st)
        if not snapshots:
            return {
                "x": st.x.copy(),
                "meas_body_vel": st.x[4:6].copy(),
            }

        t_query = float(t_query)

        if t_query <= float(snapshots[0]["t"]):
            return self._copy_snapshot_reference(snapshots[0])
        if t_query >= float(snapshots[-1]["t"]):
            return self._copy_snapshot_reference(snapshots[-1])

        for k in range(1, len(snapshots)):
            prev_entry = snapshots[k - 1]
            next_entry = snapshots[k]
            t0 = float(prev_entry["t"])
            t1 = float(next_entry["t"])
            if t0 <= t_query <= t1:
                if t1 - t0 <= 1e-12:
                    return self._copy_snapshot_reference(prev_entry)

                alpha = (t_query - t0) / (t1 - t0)

                x0 = np.asarray(prev_entry["x"], dtype=float).copy()
                x1 = np.asarray(next_entry["x"], dtype=float).copy()
                x_interp = x0 + alpha * (x1 - x0)
                dpsi = self._wrap_deg(x1[3] - x0[3])
                x_interp[3] = self._wrap_deg(x0[3] + alpha * dpsi)

                v0 = np.asarray(prev_entry["meas_body_vel"], dtype=float).reshape(2)
                v1 = np.asarray(next_entry["meas_body_vel"], dtype=float).reshape(2)
                v_interp = v0 + alpha * (v1 - v0)

                return {
                    "x": x_interp,
                    "meas_body_vel": v_interp,
                }

        return {
            "x": st.x.copy(),
            "meas_body_vel": st.x[4:6].copy(),
        }

    @staticmethod
    def _copy_snapshot_reference(entry):
        return {
            "x": np.asarray(entry["x"], dtype=float).copy(),
            "meas_body_vel": np.asarray(entry["meas_body_vel"], dtype=float).reshape(2).copy(),
        }

    def _reference_xy_at_time(self, st, t_query):
        ref = self._reference_state_at_time(st, t_query)
        if ref is None:
            return st.x[:2].copy()
        x = np.asarray(ref["x"], dtype=float).reshape(-1)
        return x[:2].copy()

    def _inertial_velocity_at_time(self, st, t_query):
        ref = self._reference_state_at_time(st, t_query)
        if ref is None:
            return self._body_to_ned_2d(st.x[4:6], st.x[3])

        x = np.asarray(ref["x"], dtype=float).reshape(-1)
        v_body = np.asarray(ref["meas_body_vel"], dtype=float).reshape(2)
        return self._body_to_ned_2d(v_body, float(x[3]))

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
                "collab_cost": float(st.collab_last_cost) if np.isfinite(st.collab_last_cost) else np.nan,
                "collab_status": str(st.collab_last_status),
                "n_collab_range": int(len(st.collab_range_window)),
                "n_collab_gps": int(len(st.collab_gps_window)),
                "collab_window_duration": float(self.window_duration),
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


DecentralizedCollabNavFilter = DecentralizedCollaborativeNavFilter
DecentralizedCollab = DecentralizedCollaborativeNavFilter
