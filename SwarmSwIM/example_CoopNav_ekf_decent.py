from SwarmSwIM import Simulator
from SwarmSwIM import activate_Acoustic, activate_Currents, save_bag
from SwarmSwIM import Visualizer2D
from utility import futils

from mac.tdma import TDMA_MAC
from mac.adaptive_mac import Adaptive_TDMA_MAC
from sensors.acoustic_ranging import AcousticRanging
from navigation.ekf_nav import EKFNavFilter
from navigation.decentralized_collab import DecentralizedCollabNavFilter

import json
from pathlib import Path

import numpy as np


#"with_ranging"
#"no_range"

ranging = True
if ranging:
    log_str = "with_ranging"
else:
    log_str = "no_range"


def save_all_logs(
    ekf_nav_logs,
    decent_nav_logs,
    compare_nav_logs,
    ekf_coop_logs,
    ekf_coop_update_debug_logs,
    decent_coop_update_debug_logs,
):
    run_log_dir = LOG_DIR / log_str
    run_log_dir.mkdir(parents=True, exist_ok=True)

    futils.save_csv(ekf_nav_logs, run_log_dir / "ekf_nav_log.csv")
    futils.save_csv(decent_nav_logs, run_log_dir / "decent_nav_log.csv")
    futils.save_csv(compare_nav_logs, run_log_dir / "compare_nav_log.csv")
    futils.save_csv(ekf_coop_logs, run_log_dir / "ekf_coop_log.csv")
    futils.save_csv(
        ekf_coop_update_debug_logs,
        run_log_dir / "ekf_coop_update_debug_log.csv",
    )
    futils.save_csv(
        decent_coop_update_debug_logs,
        run_log_dir / "decent_coop_update_debug_log.csv",
    )

# =================================
# Logging
# =================================

LOG_DIR = Path("logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)

ekf_nav_logs = []
decent_nav_logs = []
compare_nav_logs = []
ekf_coop_logs = []
ekf_coop_update_debug_logs = []
decent_coop_update_debug_logs = []

# =================================
# Simulation parameters
# =================================

ws_radius = 200

fps_physics = 30
fps_render = 30
steps_per_frame = fps_physics // fps_render

bps = 450
PDR = 1.0
c = 1500

payload_template = {
    "type": "nav",
    "id": 0,
    "tx_time": 0.0,
    "pos": [0.0, 0.0, 0.0],
    "heading": 0.0,
    "cov": [0.0, 0.0, 0.0],
    "body_vel": [0.0, 0.0],
    "decent_nav": {
        "type": "nav",
        "id": 0,
        "tx_time": 0.0,
        "pos": [0.0, 0.0, 0.0],
        "heading": 0.0,
        "cov": [0.0, 0.0, 0.0],
        "body_vel": [0.0, 0.0],
    },
}

payload_min_template = {
    "type": "consensus",
    "id": 0,
    "tx_time": 0.0,
    "q": 0.0,
    "leader_id": 0,
    "proposal_frame": 0,
    "schedule": "schedule",
}

# =================================
# Simulator
# =================================

S = Simulator(1 / fps_physics)

# save_bag(S)
activate_Currents(S)

body_vels = {
    "A01": [0.2, 0.0],
    "A02": [0.6, 0.0],
    "A03": [0.3, 0.0],
    "A04": [0.5, 0.0]
}
absolute_heading = [180, 180, 180, 180]

i = 0
for a in S.agents.values():
    i += 1
    a.set_VelocityCmd(body_vels['A0' + str(i)], mode="local_velocity")
    a.set_Heading(absolute_heading[i - 1], mode="step")

# Activate acoustic channel
ac_handle = activate_Acoustic(S, c, PDR)

# Ranging extraction module
Ranging = AcousticRanging(sound_speed=c)

# =================================
# TDMA timing
# =================================

guard_time = 2 * ws_radius / c

payload_bytes = json.dumps(payload_template).encode("utf-8")
header_bytes = 8
total_bits = (len(payload_bytes) + header_bytes) * 8

tx_duration = total_bits / bps
slot_duration = tx_duration + guard_time
frame_duration = slot_duration * len(S.agents)

frame_steps = int(frame_duration * fps_physics)

# =================================
# MAC
# =================================
MAC = TDMA_MAC(
    ac_handle,
    slot_duration=slot_duration,
    frame_duration=frame_duration,
    guard_time=guard_time
)

MAC.register_agents(S.agents.values())

# =================================
# Navigation filters initialization
# =================================

NavEKF = EKFNavFilter(writeback=True)
NavDecent = DecentralizedCollabNavFilter(
    writeback=False,
    window_duration=60.0,
    payload_key="decent_nav",
    solve_on_gps=False,
)

NavEKF.register_agents(S.agents.values())
NavDecent.register_agents(S.agents.values())

for agent in S.agents.values():
    st = NavEKF.get_state(agent)
    agent.nav_state = st
    agent.est_pos = st.x[:3].copy()
    agent.est_heading = float(st.x[3])
    agent.est_cov = st.P.copy()


def build_compare_nav_payload(agent, tx_time):
    ekf_state = NavEKF.get_state(agent)
    decent_state = NavDecent.get_state(agent)

    return {
        "type": "nav",
        "id": agent.name,
        "tx_time": tx_time,
        "pos": ekf_state.x[:3].tolist(),
        "heading": float(ekf_state.x[3]),
        "cov": np.diag(ekf_state.P[:3, :3]).tolist(),
        "body_vel": ekf_state.x[4:6].tolist(),
        "decent_nav": {
            "type": "nav",
            "id": agent.name,
            "tx_time": tx_time,
            "pos": decent_state.x[:3].tolist(),
            "heading": float(decent_state.x[3]),
            "cov": np.diag(decent_state.P[:3, :3]).tolist(),
            "body_vel": decent_state.x[4:6].tolist(),
        },
    }


def log_compare_nav_step(sim, ekf_nav, decent_nav, agents, logs):
    for agent in agents.values():
        st_ekf = ekf_nav.get_state(agent)
        st_decent = decent_nav.get_state(agent)

        err_ekf = st_ekf.x[:3] - agent.pos
        err_decent = st_decent.x[:3] - agent.pos

        logs.append({
            "t": float(sim.time),
            "agent": agent.name,

            "x": float(agent.pos[0]),
            "y": float(agent.pos[1]),
            "z": float(agent.pos[2]),

            "ekf_x_hat": float(st_ekf.x[0]),
            "ekf_y_hat": float(st_ekf.x[1]),
            "ekf_z_hat": float(st_ekf.x[2]),
            "ekf_e_norm": float(np.linalg.norm(err_ekf)),
            "ekf_traceP_pos": float(np.trace(st_ekf.P[:3, :3])),

            "decent_x_hat": float(st_decent.x[0]),
            "decent_y_hat": float(st_decent.x[1]),
            "decent_z_hat": float(st_decent.x[2]),
            "decent_e_norm": float(np.linalg.norm(err_decent)),
            "decent_traceP_pos": float(np.trace(st_decent.P[:3, :3])),
            "decent_cost": float(st_decent.collab_last_cost)
            if np.isfinite(st_decent.collab_last_cost) else np.nan,
        })

    return logs

# =================================
# Visualization properties
# =================================

properties = {
    "render_fps": fps_render,
    "bg_color": 'w',
    "grid": True,
    "color_by_type": False,
    "scale": 5.0,
    "record": False
}

# =================================
# Simulation callback
# =================================

frame = 0
next_frame_time = 0.0

def cycle(
    ekf_nav_logs,
    decent_nav_logs,
    compare_nav_logs,
    ekf_coop_logs,
    ekf_coop_update_debug_logs,
    decent_coop_update_debug_logs,
):
    global frame, next_frame_time

    # =================================
    # TDMA scheduling
    # =================================

    new_frame = False
    if S.time + 1e-9 >= next_frame_time:
        new_frame = True
        for agent in S.agents.values():
            MAC.request_tx(
                agent,
                payload_builder=build_compare_nav_payload,
                duration=tx_duration
            )
        frame += 1
        next_frame_time = frame * frame_duration

    # =================================
    # Physics step
    # =================================

    events = S.tick()

    # =================================
    # MAC step
    # =================================

    delivered = MAC(S)

    # =================================
    # Ranging extraction
    # =================================

    Ranging(S, delivered)

    # =================================
    # Navigation filter
    # =================================

    NavEKF(S, delivered)
    NavDecent(S, delivered)

    # =================================
    # Logging
    # =================================
    futils.log_nav_step(S, NavEKF, S.agents, ekf_nav_logs)
    futils.log_nav_step(S, NavDecent, S.agents, decent_nav_logs)
    log_compare_nav_step(S, NavEKF, NavDecent, S.agents, compare_nav_logs)
    futils.log_coop_events(S, delivered, NavEKF, ekf_coop_logs)
    futils.log_coop_update_debug(NavEKF, ekf_coop_update_debug_logs)
    futils.log_coop_update_debug(NavDecent, decent_coop_update_debug_logs)


    # =================================
    # Visualization bookkeeping
    # =================================

    visualizer.last_delivered = delivered

    # =================================
    # Print once per TDMA frame
    # =================================

    if new_frame:

        print(f"\n{'='*20} TDMA Frame @ t={S.time:6.2f}s {'='*20}")

        print("\n--- Positions ---")
        for agent in S.agents.values():
            print(
                f"{agent.name:>3} | "
                f"x={agent.pos[0]:8.3f}  "
                f"y={agent.pos[1]:8.3f}  "
                f"z={agent.pos[2]:6.3f}"
            )


        print("\n--- Estimation Error: EKF vs Decentralized ---")
        for agent in S.agents.values():
            st_ekf = NavEKF.get_state(agent)
            st_decent = NavDecent.get_state(agent)

            err_ekf = st_ekf.x[:3] - agent.pos
            err_decent = st_decent.x[:3] - agent.pos

            print(
                f"{agent.name:>3} | "
                f"EKF={np.linalg.norm(err_ekf):6.3f} m "
                f"(ex={err_ekf[0]:7.3f}, ey={err_ekf[1]:7.3f}) | "
                f"DEC={np.linalg.norm(err_decent):6.3f} m "
                f"(ex={err_decent[0]:7.3f}, ey={err_decent[1]:7.3f})"
            )

        frame_report = MAC.get_frame_report()

        print("\n--- Frame Report ---")
        for tx in frame_report:
            ok = len(tx["rx_success"])
            lost = len(tx["rx_lost"])

            print(
                f"TX {tx['sender']:>3} | "
                f"OK={ok:1d}  LOST={lost:1d} | "
                f"t_tx={tx['tx_time']:8.3f}"
            )

        print()

        max_age = frame_duration
        for agent in S.agents.values():
            if not hasattr(agent, "AcousticRange"):
                continue

            agent.AcousticRange = {
                k: v for k, v in agent.AcousticRange.items()
                if S.time - v["t_meas"] < max_age
            }
    

# =================================
# Run visualizer
# =================================

def cycle_callback():
    cycle(
        ekf_nav_logs,
        decent_nav_logs,
        compare_nav_logs,
        ekf_coop_logs,
        ekf_coop_update_debug_logs,
        decent_coop_update_debug_logs,
    )

visualizer = Visualizer2D(S, cycle_callback, properties, mac=MAC)

try:
    visualizer.run()
finally:
    save_all_logs(
        ekf_nav_logs,
        decent_nav_logs,
        compare_nav_logs,
        ekf_coop_logs,
        ekf_coop_update_debug_logs,
        decent_coop_update_debug_logs,
    )
