import json
import csv
from pathlib import Path

import numpy as np

def build_nav_payload(agent, tx_time):
    st = agent.nav_state

    return {
        "type": "nav",
        "id": agent.name,
        "tx_time": tx_time,
        "pos": st.x[:3].tolist(),
        "heading": float(st.x[3]),
        "cov": np.diag(st.P[:3, :3]).tolist(),
        "body_vel": st.x[4:6].tolist()
    }

def build_min_payload(agent, tx_time):
    nav_info = getattr(agent, "nav_info", {}) or {}
    traceP = float(nav_info.get("traceP_pos", 1.0))
    q_i = 1.0 / traceP if traceP > 0.0 else 0.0

    return {
        "type": "consensus",
        "id": agent.name,#default in the heade
        "tx_time": tx_time,
        "q": q_i,
    }

def log_coop_update_debug(nav, coop_update_debug_logs):
    """
    Pull accepted/rejected cooperative EKF update records from nav.coop_update_log
    into an external list, then clear the internal buffer.
    """
    if not hasattr(nav, "coop_update_log"):
        return coop_update_debug_logs

    if nav.coop_update_log:
        coop_update_debug_logs.extend(nav.coop_update_log)
        nav.coop_update_log.clear()

    return coop_update_debug_logs

def log_nav_step(sim, nav, agents, nav_logs):
    for agent in agents.values():
        st = nav.get_state(agent)

        err = st.x[:3] - agent.pos
        diagP = np.diag(st.P)

        nav_logs.append({
            "t": float(sim.time),
            "agent": agent.name,

            "x": float(agent.pos[0]),
            "y": float(agent.pos[1]),
            "z": float(agent.pos[2]),
            "psi": float(agent.psi),

            "x_hat": float(st.x[0]),
            "y_hat": float(st.x[1]),
            "z_hat": float(st.x[2]),
            "psi_hat": float(st.x[3]),
            "u_hat": float(st.x[4]),
            "v_hat": float(st.x[5]),

            "ex": float(err[0]),
            "ey": float(err[1]),
            "ez": float(err[2]),
            "e_norm": float(np.linalg.norm(err)),

            "traceP_pos": float(np.trace(st.P[:3, :3])),
            "traceP_vel": float(np.trace(st.P[4:6, 4:6])),

            "Pxx": float(diagP[0]),
            "Pyy": float(diagP[1]),
            "Pzz": float(diagP[2]),
            "Ppsi": float(diagP[3]),
            "Puu": float(diagP[4]),
            "Pvv": float(diagP[5]),
            

            "last_local_update": float(st.last_local_update),
            "last_coop_update": float(st.last_coop_update),
            "quality": float(st.quality),

        })
    return nav_logs


def log_coop_events(sim, delivered, nav, coop_logs):
    for receiver_name, msg in delivered.items():
        if msg is None or not getattr(msg, "intact", False):
            continue

        payload = getattr(msg, "payload", None)
        if payload is None:
            continue

        sender_name = msg.sender
        if sender_name is None or sender_name == receiver_name:
            continue

        receiver = sim.agents[receiver_name]
        if not hasattr(receiver, "AcousticRange"):
            continue
        if sender_name not in receiver.AcousticRange:
            continue

        meas = receiver.AcousticRange[sender_name]
        st = nav.get_state(receiver)

        payload_pos = payload.get("pos", [np.nan, np.nan, np.nan])
        payload_cov = payload.get("cov", [np.nan, np.nan, np.nan])

        if len(payload_pos) != 3:
            payload_pos = [np.nan, np.nan, np.nan]
        if len(payload_cov) != 3:
            payload_cov = [np.nan, np.nan, np.nan]

        coop_logs.append({
            "t": float(sim.time),
            "receiver": receiver_name,
            "sender": sender_name,

            "msg_intact": int(bool(msg.intact)),
            "range": float(meas.get("range", np.nan)),
            "t_meas": float(meas.get("t_meas", np.nan)),
            "t_tx_payload": float(payload.get("tx_time", np.nan)),
            "packet_age": float(meas.get("t_meas", np.nan) - payload.get("tx_time", np.nan)),

            "sender_x_hat": float(payload_pos[0]),
            "sender_y_hat": float(payload_pos[1]),
            "sender_z_hat": float(payload_pos[2]),

            "sender_Pxx": float(payload_cov[0]),
            "sender_Pyy": float(payload_cov[1]),
            "sender_Pzz": float(payload_cov[2]),

            "receiver_x_hat": float(st.x[0]),
            "receiver_y_hat": float(st.x[1]),
            "receiver_z_hat": float(st.x[2]),
            "receiver_traceP_pos": float(np.trace(st.P[:3, :3])),
            "nu" : float(meas.get("nu", np.nan)),
        })
        
    return coop_logs

def save_csv(rows, filepath):
    if not rows:
        print(f"No rows to save for {filepath}")
        return

    fieldnames = list(rows[0].keys())
    with open(filepath, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved: {filepath}")


from pathlib import Path
import numpy as np


def save_all_logs(nav_logs, coop_logs, coop_update_debug_logs, log_dir, log_str):
    """
    Save all simulation logs using the existing save_csv() function.
    """
    log_dir = Path(log_dir)

    save_csv(nav_logs, log_dir / log_str / "nav_log.csv")
    save_csv(coop_logs, log_dir / log_str / "coop_log.csv")
    save_csv(
        coop_update_debug_logs,
        log_dir / log_str / "coop_update_debug_log.csv",
    )


def initialize_nav_agent_fields(sim, nav):
    """
    Initialize agent-side navigation fields from the navigation filter.
    """
    for agent in sim.agents.values():
        st = nav.get_state(agent)
        agent.nav_state = st
        agent.est_pos = st.x[:3].copy()
        agent.est_heading = float(st.x[3])
        agent.est_cov = st.P.copy()


def update_sporadic_gps_fixes(
    sim,
    nav,
    base_surface_agents,
    gps_fix_until,
    gps_fix_last,
    enabled=False,
    duration=1.0,
    cooldown=300.0,
    risk_thresh=1.0,
    traceP_ref=2.0,
    abs_age_ref=300.0,
    coop_age_ref=60.0,
    nis_ref=9.0,
    w_traceP=0.55,
    w_abs_age=0.25,
    w_coop_age=0.20,
    w_nis=0.00,
    verbose=True,
):
    """
    Temporarily add high-risk underwater agents to nav.surface_agents.
    """
    if not enabled:
        nav.surface_agents = set(base_surface_agents)
        return

    t = float(sim.time)
    active_surface_agents = set(base_surface_agents)

    for agent in sim.agents.values():
        nav_info = getattr(agent, "nav_info", {}) or {}

        traceP = float(nav_info.get("traceP_pos", np.inf))
        traceP_term = (
            traceP / traceP_ref
            if np.isfinite(traceP) and traceP_ref > 0.0
            else np.inf
        )

        active_until = gps_fix_until.get(agent.name, -np.inf)
        last_fix = gps_fix_last.get(agent.name, -np.inf)

        if t <= active_until:
            active_surface_agents.add(agent.name)
            continue

        if agent.name in base_surface_agents:
            continue

        abs_age = t - last_fix if np.isfinite(last_fix) else t
        abs_age_term = abs_age / abs_age_ref if abs_age_ref > 0.0 else 0.0

        last_coop = float(nav_info.get("last_coop_update", -np.inf))
        coop_age = t - last_coop if np.isfinite(last_coop) else t
        coop_age_term = coop_age / coop_age_ref if coop_age_ref > 0.0 else 0.0

        nis_val = nav_info.get("avg_NIS", nav_info.get("latest_NIS", None))
        if nis_val is None:
            nis_term = 0.0
        else:
            nis_val = float(nis_val)
            nis_term = (
                nis_val / nis_ref
                if np.isfinite(nis_val) and nis_ref > 0.0
                else 0.0
            )

        risk = (
            w_traceP * traceP_term
            + w_abs_age * abs_age_term
            + w_coop_age * coop_age_term
            + w_nis * nis_term
        )

        if risk >= risk_thresh and t - last_fix >= cooldown:
            active_surface_agents.add(agent.name)
            gps_fix_until[agent.name] = t + duration
            gps_fix_last[agent.name] = t

            if verbose:
                q_i = 1.0 / traceP if traceP > 0.0 and np.isfinite(traceP) else 0.0

                print(
                    f"[GPS FIX] t={t:.2f}s | {agent.name} temporarily promoted "
                    f"to surface agent | risk={risk:.3f} | "
                    f"traceP={traceP:.4f} | q={q_i:.4f} | "
                    f"abs_age={abs_age:.1f}s | coop_age={coop_age:.1f}s | "
                    f"nis_term={nis_term:.3f}"
                )

    nav.surface_agents = active_surface_agents


def prune_acoustic_ranges(sim, max_age):
    """
    Remove old AcousticRange entries.
    """
    for agent in sim.agents.values():
        if not hasattr(agent, "AcousticRange"):
            continue

        agent.AcousticRange = {
            k: v for k, v in agent.AcousticRange.items()
            if sim.time - v["t_meas"] < max_age
        }


def print_adaptive_frame_report(
    sim,
    nav,
    mac,
    base_surface_agents,
    frame_report=None,
    print_geometry=True,
):
    """
    Print one adaptive-frame diagnostic report.
    """
    print(f"\n{'=' * 20} Adaptive Frame @ t={sim.time:6.2f}s - ID {mac.frame_id} {'=' * 20}")

    print("\n--- Positions ---")
    for agent in sim.agents.values():
        print(
            f"{agent.name:>3} | "
            f"x={agent.pos[0]:8.3f}  "
            f"y={agent.pos[1]:8.3f}  "
            f"z={agent.pos[2]:6.3f}"
        )

    print("\n--- Estimation Error ---")
    for agent in sim.agents.values():
        if not hasattr(agent, "nav_state"):
            continue

        st = agent.nav_state
        err = st.x[:3] - agent.pos

        print(
            f"{agent.name:>3} | "
            f"x_hat={np.array2string(st.x[:6], precision=3, floatmode='fixed', suppress_small=True, separator=' ')}"
            f"ex={err[0]:7.3f}  "
            f"ey={err[1]:7.3f}  "
            f"ez={err[2]:7.3f}  "
            f"| norm={np.linalg.norm(err):6.3f}"
        )

    print("\n--- GPS Fix State ---")
    for agent in sim.agents.values():
        nav_info = getattr(agent, "nav_info", {}) or {}
        traceP = float(nav_info.get("traceP_pos", np.inf))
        q_i = 1.0 / traceP if traceP > 0.0 and np.isfinite(traceP) else 0.0
        is_surface = agent.name in nav.surface_agents
        is_base_surface = agent.name in base_surface_agents

        print(
            f"{agent.name:>3} | "
            f"q={q_i:8.4f} | "
            f"traceP={traceP:8.4f} | "
            f"surface={is_surface} | "
            f"base_surface={is_base_surface}"
        )

    print(f"\n--- MAC stats ---")
    print(
        f"TX={mac.stats['tx']:3d} | "
        f"RX_OK={mac.stats['rx_success']:3d} | "
        f"RX_LOST={mac.stats['rx_lost']:3d} | "
        f"COLL={mac.stats['collisions']:3d} | "
        f"DENIED={mac.stats['denied']:3d}"
    )

    if frame_report is None:
        frame_report = mac.get_frame_report()

    for tx in frame_report:
        ok = len(tx["rx_success"])
        lost = len(tx["rx_lost"])
        print(
            f"TX {tx['sender']:>3} | "
            f"OK={ok:1d}  LOST={lost:1d} | "
            f"t_tx={tx['tx_time']:8.3f}"
        )

    print(f"\nModes: {mac.active_modes}")
    print(f"Frame duration: {mac.frame_duration:.3f}s")
    print(f"Frame start: {mac.frame_start_time:.3f}s")

    if print_geometry and hasattr(mac, "_compute_information_gain"):
        selected = [n for n, m in mac.active_modes.items() if m == "nav"]

        print("\n--- Marginal geometric relevance ---")
        for name in mac.agents_order:
            I_without = mac._compute_information_gain(
                [n for n in selected if n != name],
                sim,
            )
            I_with = mac._compute_information_gain(
                list(set(selected) | {name}),
                sim,
            )
            dI = I_with - I_without
            mode = mac.active_modes.get(name, "min")

            print(
                f"{name:>3} | "
                f"mode={mode:>3} | "
                f"dI_team={dI:8.4f}"
            )

    print()


def finalize_and_exit(
    sim,
    nav_logs,
    coop_logs,
    coop_update_debug_logs,
    log_dir,
    log_str,
):
    """
    Save logs and stop the simulation.
    """
    save_all_logs(
        nav_logs,
        coop_logs,
        coop_update_debug_logs,
        log_dir,
        log_str,
    )

    print(f"[STOP] Simulation reached t={sim.time:.2f}s")
    exit()

