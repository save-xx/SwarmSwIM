import numpy as np
import casadi
from dataclasses import dataclass, field

from .base_nav import BaseNavFilter


@dataclass
class NavState:
    x: np.ndarray
    P: np.ndarray
    t: float

    x0_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))

    last_local_update: float = 0.0
    last_coop_update: float = 0.0
    quality: float = np.inf

    last_coop_meas_time: dict = field(default_factory=dict)
    history: list = field(default_factory=list)

    fg_range_window: list = field(default_factory=list)
    fg_gps_window: list = field(default_factory=list)

    fg_last_solution: dict = field(default_factory=dict)
    fg_last_status: str = ""
    fg_last_cost: float = np.nan


class FGNavFilter(BaseNavFilter):

    def __init__(
        self,
        Q_diag=(0.01, 0.01, 0.0001, 0.03, 0.01, 0.02),
        P0_diag=(0.25, 0.25, 1e-4, 9.0, 0.01, 0.01),
        R_depth=1e-6,
        R_heading_deg=4.0,
        R_body_vel_diag=np.array([0.003, 0.003]),
        surface_agents=("A04", "A02"),
        var_gps_fix=np.array([0.25, 0.25]),
        history_length=1000,
        writeback=True,
        keep_history=True,
        rng_seed=50,
        fg_time_horizon=10.0,
        max_range_events=2000000,
        max_gps_events=20000000,
        min_fg_nodes=2,
        fg_sigma_prior=0.5,
        fg_sigma_v=0.1,
        fg_sigma_range=0.1,
        fg_sigma_gps=0.5,
    ):
        super().__init__()

        self.Q_diag = np.asarray(Q_diag, dtype=float)
        self.P0_diag = np.asarray(P0_diag, dtype=float)

        self.R_depth = float(R_depth)
        self.R_heading = float(R_heading_deg)
        self.R_body_vel = np.diag(np.asarray(R_body_vel_diag, dtype=float))

        self.var_gps_fix = np.diag(np.asarray(var_gps_fix, dtype=float))
        self.surface_agents = set(surface_agents)

        self.history_length = int(history_length)
        self.writeback = bool(writeback)
        self.keep_history = bool(keep_history)

        self.fg_time_horizon = float(fg_time_horizon)
        self.max_range_events = int(max_range_events)
        self.max_gps_events = int(max_gps_events)
        self.min_fg_nodes = int(min_fg_nodes)

        self.fg_sigma_prior = float(fg_sigma_prior)
        self.fg_sigma_v = float(fg_sigma_v)
        self.fg_sigma_range = float(fg_sigma_range)
        self.fg_sigma_gps = float(fg_sigma_gps)

        self.rng = np.random.default_rng(rng_seed)
        self.fg_last_cost = np.nan

    # ==========================================================
    # INIT
    # ==========================================================

    def _init_filter(self, agent):
        vel0 = self._get_body_velocity_measurement(agent, None)

        x0 = np.array([
            float(agent.pos[0]),
            float(agent.pos[1]),
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
            x0_xy=x0[:2].copy(),
            quality=float(np.trace(P0[:3, :3])),
        )

        self._append_history_snapshot(st, 0.0, st.x)
        return st

    # ==========================================================
    # PREDICT / LOCAL
    # ==========================================================

    def predict(self, agent, sim):
        st = self.filters[agent.name]

        dt = float(sim.time - st.t)
        if dt <= 0.0:
            return

        x = st.x.copy()

        psi = float(x[3])
        u = float(x[4])
        v = float(x[5])

        vel_ned = self._body_to_ned_2d(np.array([u, v]), psi)

        x[0] += vel_ned[0] * dt
        x[1] += vel_ned[1] * dt
        x[3] = self._wrap_deg(x[3])

        st.x = x
        st.t = float(sim.time)

        self._append_history_snapshot(st, float(sim.time), st.x)

    def update_local(self, agent, sim):
        st = self.filters[agent.name]

        z_depth = float(agent.measured_depth)
        z_psi = float(agent.measured_heading)
        z_vel = self._get_body_velocity_measurement(agent, sim)

        st.x[2] = z_depth
        st.x[3] = self._wrap_deg(z_psi)
        st.x[4:6] = np.asarray(z_vel, dtype=float).reshape(2)

        st.last_local_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))

        self._append_history_snapshot(st, float(sim.time), st.x)

    # ==========================================================
    # GPS
    # ==========================================================

    def update_surface_position(self, agent, sim):
        if not self._is_surface_agent(agent):
            return

        st = self.filters[agent.name]

        if self._add_gps_event(agent, sim):
            result = self._solve_agent_fg(st, float(sim.time), use_gps=True)
            self._apply_fg_solution(st, result, sim)

        if self.writeback:
            self._writeback_all(sim)

    def _add_gps_event(self, agent, sim):
        st = self.filters[agent.name]
        t = float(sim.time)

        if st.fg_gps_window:
            if abs(float(st.fg_gps_window[-1]["t"]) - t) < 1e-12:
                return False

        gps_noise = self.rng.normal(
            loc=0.0,
            scale=np.sqrt(np.diag(self.var_gps_fix)),
            size=2,
        )

        z = np.array([
            float(agent.pos[0]) + gps_noise[0],
            float(agent.pos[1]) + gps_noise[1],
        ], dtype=float)

        if not np.all(np.isfinite(z)):
            return False

        st.fg_gps_window.append({"t": t, "z": z.copy()})

        if len(st.fg_gps_window) > self.max_gps_events:
            st.fg_gps_window[:] = st.fg_gps_window[-self.max_gps_events:]

        return True

    # ==========================================================
    # COOP RANGE
    # ==========================================================

    def process_cooperative(self, sim, delivered):
        for receiver_name, msg in delivered.items():

            if msg is None or not getattr(msg, "intact", False):
                continue

            if msg.sender is None or msg.sender == receiver_name:
                continue

            receiver = sim.agents[receiver_name]

            if not hasattr(receiver, "AcousticRange"):
                continue

            if msg.sender not in receiver.AcousticRange:
                continue

            if msg.payload is None or msg.payload.get("type") != "nav":
                continue

            st = self.filters[receiver_name]
            meas = receiver.AcousticRange[msg.sender]

            t_meas = meas.get("t_meas", None)
            if t_meas is None:
                continue

            t_meas = float(t_meas)

            last_t = st.last_coop_meas_time.get(msg.sender, -np.inf)
            if t_meas <= last_t:
                continue

            event = self._build_range_event(msg.payload, meas)
            if event is None:
                continue

            self._append_range_event(st, event)

            result = self._solve_agent_fg(
                st,
                float(sim.time),
                use_gps=(receiver_name in self.surface_agents),
            )
            self._apply_fg_solution(st, result, sim)

            st.last_coop_meas_time[msg.sender] = t_meas

        if self.writeback:
            self._writeback_all(sim)

    def _build_range_event(self, payload, meas):
        if "range" not in meas:
            return None

        if "pos" not in payload:
            return None

        t_meas = meas.get("t_meas", None)
        if t_meas is None:
            return None

        t_meas = float(t_meas)
        t_tx = float(payload.get("tx_time", meas.get("t_tx", t_meas)))

        z_range = float(meas["range"])
        if not np.isfinite(z_range):
            return None

        p_j = np.asarray(payload["pos"], dtype=float).reshape(-1)
        if p_j.size < 2 or not np.all(np.isfinite(p_j[:2])):
            return None

        psi_j = float(payload.get("heading", 0.0))
        if not np.isfinite(psi_j):
            psi_j = 0.0

        v_j = np.asarray(payload.get("body_vel", [0.0, 0.0]), dtype=float).reshape(-1)
        if v_j.size != 2 or not np.all(np.isfinite(v_j)):
            v_j = np.zeros(2, dtype=float)

        return {
            "t": t_meas,
            "t_meas": t_meas,
            "t_tx": t_tx,
            "z": z_range,
            "p_j_tx": p_j[:2].copy(),
            "psi_j": psi_j,
            "v_j": v_j.copy(),
        }

    def _append_range_event(self, st, ev):
        st.fg_range_window.append(ev)

        if len(st.fg_range_window) > self.max_range_events:
            st.fg_range_window[:] = st.fg_range_window[-self.max_range_events:]

    # ==========================================================
    # SOLVER
    # ==========================================================

    def _solve_agent_fg(self, st, t_now, use_gps=True):
        range_events, gps_events = self._select_recent_events(st, t_now)

        if not use_gps:
            gps_events = []

        t_now = float(t_now)

        times = self._build_node_times(
            st=st,
            t_now=t_now,
            range_events=range_events,
            gps_events=gps_events,
        )

        if len(times) < self.min_fg_nodes:
            return None

        #time_to_idx = {t: k for k, t in enumerate(times)}
        N = len(times)

        X = casadi.SX.sym("X", 2 * N)

        def xk(k):
            return X[2 * k: 2 * k + 2]

        J = casadi.SX(0)

        g = []
        lbg = []
        ubg = []

        v_max = 1.0
        d_min = 3.0

        ref = [self._reference_xy_at_time(st, t) for t in times]

        # Keep the original initial-position anchor.
        # This is not a moving anchor at the start of the current window.
        prior_xy = st.x0_xy.copy()

        r0 = (xk(0) - prior_xy) / self.fg_sigma_prior
        J += casadi.dot(r0, r0)

        # Velocity-integration residual + max-displacement constraint
        for k in range(1, N):
            dt = max(1e-9, float(times[k] - times[k - 1]))

            delta_dr = self._integrate_dr_between(
                st,
                times[k - 1],
                times[k],
            )

            step = xk(k) - xk(k - 1)

            r = (step - delta_dr) / (self.fg_sigma_v * dt)
            J += casadi.dot(r, r)

            step_norm = casadi.sqrt(casadi.dot(step, step) + 1e-12)

            # Hard constraint:
            # ||x_k - x_{k-1}|| <= v_max * dt
            g.append(step_norm - v_max * dt)
            lbg.append(-casadi.inf)
            ubg.append(0.0)

        # Range residual + minimum-distance constraint
        for ev in range_events:
            #k = time_to_idx[ev["t"]]
            k = self._nearest_time_index(times, ev["t"])
            p_j = self._sender_xy_at_meas(ev)

            dx = xk(k)[0] - float(p_j[0])
            dy = xk(k)[1] - float(p_j[1])

            dist = casadi.sqrt(dx * dx + dy * dy + 1e-12)

            r = (dist - float(ev["z"])) / self.fg_sigma_range
            J += r * r

            # Hard constraint:
            # ||x_i - x_j|| >= d_min
            g.append(dist - d_min)
            lbg.append(0.0)
            ubg.append(casadi.inf)

        # Absolute GPS fix residuals
        for ev in gps_events:
            k = self._nearest_time_index(times, ev["t"])
            z = np.asarray(ev["z"], dtype=float).reshape(2)

            r = (xk(k) - z) / self.fg_sigma_gps
            J += casadi.dot(r, r)

        x0 = self._build_initial_guess(st, times, ref)

        try:
            nlp = {
                "x": X,
                "f": J,
                "g": casadi.vertcat(*g) if len(g) > 0 else casadi.SX.zeros(0, 1),
            }

            solver = casadi.nlpsol(
                f"fg_solver_{id(st)}_{N}_{len(st.fg_range_window)}_{len(st.fg_gps_window)}",
                "ipopt",
                nlp,
                {
                    "print_time": 0,
                    "verbose": False,
                    "ipopt.print_level": 0,
                    "ipopt.sb": "yes",
                },
            )

            sol = solver(
                x0=x0,
                lbg=lbg,
                ubg=ubg,
            )

            stats = solver.stats()
            success = bool(stats.get("success", True))
            status = str(stats.get("return_status", ""))

            x_opt = np.asarray(sol["x"].full()).reshape(-1)
            cost = float(sol["f"])

        except Exception as exc:
            return {
                "success": False,
                "xy_last": None,
                "xy_all": None,
                "node_times": times,
                "cost": np.inf,
                "status": str(exc),
            }

        xy_all = [
            x_opt[2 * k: 2 * k + 2].copy()
            for k in range(N)
        ]

        return {
            "success": success,
            "xy_last": xy_all[-1].copy(),
            "xy_all": xy_all,
            "node_times": times,
            "cost": cost,
            "status": status,
        }

    def _apply_fg_solution(self, st, result, sim):
        if result is None or not result.get("success", False):
            return

        xy_last = np.asarray(result["xy_last"], dtype=float).reshape(2)

        if not np.all(np.isfinite(xy_last)):
            return

        st.x[0:2] = xy_last
        st.t = float(sim.time)
        st.last_coop_update = float(sim.time)
        st.quality = float(np.trace(st.P[:3, :3]))

        st.fg_last_solution = {
            "node_times": [float(t) for t in result["node_times"]],
            "xy_all": [
                np.asarray(xy, dtype=float).reshape(2).copy()
                for xy in result["xy_all"]
            ],
        }

        st.fg_last_cost = float(result.get("cost", np.nan))
        st.fg_last_status = str(result.get("status", ""))

        self.fg_last_cost = st.fg_last_cost

        self._append_history_snapshot(st, float(sim.time), st.x)

    def _select_recent_events(self, st, t_now):
        t_min = float(t_now) - self.fg_time_horizon

        range_events = [
            ev for ev in st.fg_range_window
            if float(ev["t"]) >= t_min
        ]

        gps_events = [
            ev for ev in st.fg_gps_window
            if float(ev["t"]) >= t_min
        ]

        if len(range_events) > self.max_range_events:
            range_events = range_events[-self.max_range_events:]

        if len(gps_events) > self.max_gps_events:
            gps_events = gps_events[-self.max_gps_events:]

        return range_events, gps_events

    def _build_node_times(self, st, t_now, range_events, gps_events):
        """
        Sliding-window node times.

        Includes:
        - true initial anchor time 0.0 when still inside the horizon;
        - otherwise the window start time;
        - all range-event times;
        - all GPS-event times;
        - current time.
        """
        t_now = float(t_now)
        t_start = max(0.0, t_now - self.fg_time_horizon)

        times = [t_start, t_now]

        for ev in range_events:
            times.append(float(ev["t"]))

        for ev in gps_events:
            times.append(float(ev["t"]))

        return self._unique_sorted_times(times)

    @staticmethod
    def _unique_sorted_times(times, tol=1e-9):
        out = []

        for t in sorted(float(v) for v in times if np.isfinite(float(v))):
            if not out or abs(t - out[-1]) > tol:
                out.append(t)

        return out

    def _build_initial_guess(self, st, times, ref):
        """
        Warm start:
        1. interpolate previous optimized FG trajectory if available;
        2. otherwise fall back to recursive DR/history reference.
        """
        x0 = []
        prev = st.fg_last_solution if st.fg_last_solution else None

        for k, t in enumerate(times):
            xy = None

            if prev:
                xy = self._interpolate_solution(prev, float(t))

            if xy is None:
                xy = np.asarray(ref[k], dtype=float).reshape(2)

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
    # DR / HISTORY
    # ==========================================================

    def _append_history_snapshot(self, st, t, x):
        if not self.keep_history:
            return

        st.history.append({
            "t": float(t),
            "x": np.asarray(x, dtype=float).copy(),
        })

        if len(st.history) > self.history_length:
            st.history[:] = st.history[-self.history_length:]

    def _reference_state_at_time(self, st, t_query):
        """
        Interpolate full recursive state at t_query using history.

        Interpolates:
        - x, y, z linearly
        - psi with angle wrapping
        - u, v linearly
        """
        t_query = float(t_query)

        if not st.history:
            return st.x.copy()

        hist = sorted(st.history, key=lambda e: float(e["t"]))

        if t_query <= float(hist[0]["t"]):
            return np.asarray(hist[0]["x"], dtype=float).copy()

        if t_query >= float(hist[-1]["t"]):
            return np.asarray(hist[-1]["x"], dtype=float).copy()

        for k in range(1, len(hist)):
            e0 = hist[k - 1]
            e1 = hist[k]

            t0 = float(e0["t"])
            t1 = float(e1["t"])

            if t0 <= t_query <= t1:
                x0 = np.asarray(e0["x"], dtype=float).copy()
                x1 = np.asarray(e1["x"], dtype=float).copy()

                if t1 - t0 <= 1e-12:
                    return x0.copy()

                alpha = (t_query - t0) / (t1 - t0)

                x = x0 + alpha * (x1 - x0)

                dpsi = self._wrap_deg(x1[3] - x0[3])
                x[3] = self._wrap_deg(x0[3] + alpha * dpsi)

                return x

        return st.x.copy()

    def _reference_xy_at_time(self, st, t_query):
        x = self._reference_state_at_time(st, t_query)
        return np.asarray(x[:2], dtype=float).copy()

    def _integrate_dr_between(self, st, t0, t1):
        """
        DR displacement between t0 and t1 from interpolated recursive history.
        """
        p0 = self._reference_xy_at_time(st, t0)
        p1 = self._reference_xy_at_time(st, t1)

        return p1 - p0

    # ==========================================================
    # TIMING / SENDER PROPAGATION
    # ==========================================================

    def _sender_xy_at_meas(self, ev):
        p = np.asarray(ev["p_j_tx"], dtype=float).reshape(2).copy()

        dt = max(0.0, float(ev["t_meas"]) - float(ev["t_tx"]))
        vel = self._body_to_ned_2d(ev["v_j"], ev["psi_j"])

        return p + vel * dt

    # ==========================================================
    # HELPERS
    # ==========================================================

    @staticmethod
    def _nearest_time_index(times, t_query):
        arr = np.asarray(times, dtype=float)
        return int(np.argmin(np.abs(arr - float(t_query))))

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
                "fg_cost": float(st.fg_last_cost)
                if np.isfinite(st.fg_last_cost)
                else np.nan,
                "fg_status": str(st.fg_last_status),
                "n_fg_range": int(len(st.fg_range_window)),
                "n_fg_gps": int(len(st.fg_gps_window)),
            }

    def _is_surface_agent(self, agent):
        return agent.name in self.surface_agents

    def _get_body_velocity_measurement(self, agent, sim):
        if hasattr(agent, "emulated_velocities"):
            return np.asarray(agent.emulated_velocities, dtype=float).reshape(2).copy()
        return np.zeros(2, dtype=float)

    @staticmethod
    def _body_to_ned_2d(v, psi):
        psi = np.deg2rad(psi)
        R = np.array([
            [np.cos(psi), np.sin(psi)],
            [np.sin(psi), -np.cos(psi)],
        ])
        return R @ np.asarray(v, dtype=float).reshape(2)

    @staticmethod
    def _wrap_deg(a):
        return (float(a) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _symmetrize(M):
        return 0.5 * (M + M.T)