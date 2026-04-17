import json
import csv
from pathlib import Path

import numpy as np

def build_nav_payload(agent, tx_time):
    st = agent.nav_state

    return {
        "id": agent.name,
        "tx_time": tx_time,
        "pos": st.x[:3].tolist(),
        "heading": float(st.x[3]),
        "cov": np.diag(st.P[:3, :3]).tolist()
    }

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




