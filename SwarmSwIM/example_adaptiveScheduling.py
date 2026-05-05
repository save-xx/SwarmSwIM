from SwarmSwIM import Simulator
from SwarmSwIM import activate_Acoustic, activate_Currents
from SwarmSwIM import Visualizer2D
from utility import futils

from mac.adaptive_mac import Adaptive_TDMA_MAC
from sensors.acoustic_ranging import AcousticRanging
from navigation.ekf_nav import EKFNavFilter
from navigation.fg_nav import FGNavFilter

import json
from pathlib import Path

import numpy as np


ranging = True
log_str = "with_ranging" if ranging else "no_range"

leader_id = "A04"
K_select = 2


def save_all_logs(nav_logs, coop_logs, coop_update_debug_logs):
    futils.save_csv(nav_logs, LOG_DIR / log_str / "nav_log.csv")
    futils.save_csv(coop_logs, LOG_DIR / log_str / "coop_log.csv")
    futils.save_csv(coop_update_debug_logs, LOG_DIR / log_str / "coop_update_debug_log.csv")


# =================================
# Logging
# =================================

LOG_DIR = Path("logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)

nav_logs = []
coop_logs = []
coop_update_debug_logs = []

# =================================
# Simulation parameters
# =================================

ws_radius = 200

fps_physics = 30
fps_render = 30
bps = 450
PDR = 0.7
c = 1500

payload_nav_template = {
    "type": "nav",
    "id": 0,
    "tx_time": 0.0,
    "pos": [0.0, 0.0, 0.0],
    "heading": 0.0,
    "cov": [0.0, 0.0, 0.0],
    "body_vel": [0.0, 0.0],
}

payload_min_template = {
    "type": "consensus",
    "id": 0,
    "tx_time": 0.0,
    "q": 0.0,
}

# =================================
# Simulator
# =================================

S = Simulator(1 / fps_physics)
activate_Currents(S)

body_vels = {
    "A01": [0.2, 0.0],
    "A02": [0.6, 0.0],
    "A03": [0.3, 0.0],
    "A04": [0.5, 0.0],
}
absolute_heading = [180, 180, 180, 180]

i = 0
for a in S.agents.values():
    i += 1
    a.set_VelocityCmd(body_vels[f"A0{i}"], mode="local_velocity")
    a.set_Heading(absolute_heading[i - 1], mode="step")

# =================================
# Acoustic and ranging
# =================================

ac_handle = activate_Acoustic(S, c, PDR)
Ranging = AcousticRanging(sound_speed=c)

# =================================
# Packet durations
# =================================

guard_time = 2 * ws_radius / c
header_bytes = 8

payload_nav_bytes = json.dumps(payload_nav_template).encode("utf-8")
total_nav_bits = (len(payload_nav_bytes) + header_bytes) * 8
tx_nav_duration = total_nav_bits / bps

payload_min_bytes = json.dumps(payload_min_template).encode("utf-8")
total_min_bits = (len(payload_min_bytes) + header_bytes) * 8
tx_min_duration = total_min_bits / bps

# =================================
# MAC
# =================================

MAC = Adaptive_TDMA_MAC(
    ac_handle,
    nav_duration=tx_nav_duration,
    min_duration=tx_min_duration,
    guard_time=guard_time,
    leader_id=leader_id,
    nav_payload_builder=futils.build_nav_payload,
    min_payload_builder=futils.build_min_payload,
    K_select=K_select,
)

MAC.register_agents(S.agents.values())

# =================================
# Navigation filters
# =================================

#Nav = EKFNavFilter(writeback=True)
Nav = FGNavFilter()

Nav.register_agents(S.agents.values())

for agent in S.agents.values():
    st = Nav.get_state(agent)
    agent.nav_state = st
    agent.est_pos = st.x[:3].copy()
    agent.est_heading = float(st.x[3])
    agent.est_cov = st.P.copy()

# =================================
# Visualization properties
# =================================

properties = {
    "render_fps": fps_render,
    "bg_color": "w",
    "grid": True,
    "color_by_type": False,
    "scale": 5.0,
    "record": False,
}

# =================================
# Initial bootstrap frame
# =================================

for agent in S.agents.values():
    MAC.request_tx(
        agent,
        payload_builder=futils.build_nav_payload,
        duration=tx_nav_duration,
    )

frame = 1
print_bootstrap = True

# =================================
# Navigation update rates
# =================================

local_update_hz = 5.0
gps_update_hz = 1.0

local_update_dt = 1.0 / local_update_hz
gps_update_dt = 1.0 / gps_update_hz

next_local_update_time = 0.0
next_gps_update_time = 0.0
# =================================
# Simulation callback
# =================================

def cycle(nav_logs, coop_logs, coop_update_debug_logs):
    global frame, print_bootstrap, next_local_update_time, next_gps_update_time

    prev_mac_frame_id = getattr(MAC, "frame_id", None)

    # =================================
    # Physics step
    # =================================

    S.tick()

    # =================================
    # MAC step
    # =================================

    delivered = MAC(S)

    # =================================
    # Detect adaptive frame transition
    # =================================

    new_frame = False
    curr_mac_frame_id = getattr(MAC, "frame_id", None)

    if print_bootstrap:
        new_frame = True
        print_bootstrap = False
    elif prev_mac_frame_id is not None and curr_mac_frame_id is not None:
        if curr_mac_frame_id != prev_mac_frame_id:
            new_frame = True

            for agent in S.agents.values():
                MAC.request_tx(
                    agent,
                    payload_builder=futils.build_nav_payload,
                    duration=tx_nav_duration,
                )

            frame += 1

    # =================================
    # Ranging extraction
    # =================================

    Ranging(S, delivered)

    # =================================
    # Navigation filter
    # =================================

    # Predict every simulator cycle
    for agent in S.agents.values():
        Nav.predict(agent, S)

    # Cooperative every cycle
    Nav.process_cooperative(S, delivered)

    # Common local updates at 10 Hz
    if S.time + 1e-9 >= next_local_update_time:
        for agent in S.agents.values():
            Nav.update_local(agent, S)
        next_local_update_time += local_update_dt

    # GPS/surface XY at 1 Hz
    if S.time + 1e-9 >= next_gps_update_time:
        for agent in S.agents.values():
            Nav.update_surface_position(agent, S)
        next_gps_update_time += gps_update_dt

    # =================================
    # Logging
    # =================================

    futils.log_nav_step(S, Nav, S.agents, nav_logs)
    futils.log_coop_events(S, delivered, Nav, coop_logs)
    futils.log_coop_update_debug(Nav, coop_update_debug_logs)

    # =================================
    # Visualization bookkeeping
    # =================================

    visualizer.last_delivered = delivered

    # =================================
    # Print once per adaptive frame
    # =================================

    if new_frame:
        print(f"\n{'='*20} Adaptive Frame @ t={S.time:6.2f}s {'='*20}")

        print("\n--- Positions ---")
        for agent in S.agents.values():
            print(
                f"{agent.name:>3} | "
                f"x={agent.pos[0]:8.3f}  "
                f"y={agent.pos[1]:8.3f}  "
                f"z={agent.pos[2]:6.3f}"
            )

        print("\n--- Estimation Error ---")
        for agent in S.agents.values():
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

        print("\n--- MAC stats (adaptive) ---")
        print(
            f"TX={MAC.stats['tx']:3d} | "
            f"RX_OK={MAC.stats['rx_success']:3d} | "
            f"RX_LOST={MAC.stats['rx_lost']:3d} | "
            f"COLL={MAC.stats['collisions']:3d} | "
            f"DENIED={MAC.stats['denied']:3d}"
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

        print(f"\nModes: {MAC.active_modes}")
        print(f"Frame duration: {MAC.frame_duration:.3f}s")
        print(f"Frame start: {MAC.frame_start_time:.3f}s")
        print(f"Frame id: {MAC.frame_id}")

        print()

        max_age = MAC.frame_duration
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
    cycle(nav_logs, coop_logs, coop_update_debug_logs)

visualizer = Visualizer2D(S, cycle_callback, properties, mac=MAC)

try:
    visualizer.run()
finally:
    save_all_logs(nav_logs, coop_logs, coop_update_debug_logs)