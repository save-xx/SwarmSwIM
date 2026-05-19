from pathlib import Path
import json

import numpy as np

from SwarmSwIM import Simulator
from SwarmSwIM import activate_Acoustic, activate_Currents
from SwarmSwIM import Visualizer2D

from utility import utils
from mac.adaptive_mac import Adaptive_TDMA_MAC
from sensors.acoustic_ranging import AcousticRanging
from navigation.ekf_nav import EKFNavFilter
from navigation.fg_nav import FGNavFilter


# =================================
# Experiment configuration
# =================================

ranging = True

heading_bias = True
if not ranging:
    policy="DR"
else:
    policy="tdma"
    #policy="trivial"
    #policy="adaptive" 

log_str = "with_ranging" if ranging else "no_range"

leader_id = "A02"
K_select = 4

LOG_DIR = Path("logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)

SIM_STOP_TIME = 700.0

ws_radius = 200
fps_physics = 30
fps_render = 30
bps = 450

pdr_sender = {
    "A01": 0.6,
    "A02": 0.6,   # surface/reference, reliable
    "A03": 0.6,   # geometrically useful but unreliable
    "A04": 0.6,   # geometrically useful but unreliable
}
c = 1500


# =================================
# Payload templates
# =================================

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
# Logs
# =================================

nav_logs = []
coop_logs = []
coop_update_debug_logs = []


# =================================
# Simulator
# =================================

S = Simulator(1 / fps_physics)
activate_Currents(S)

body_vels = {
    "A01": [0.2, 0.0],
    "A02": [0.5, 0.0],
    "A03": [0.3, 0.0],
    "A04": [0.4, 0.0],
}


# Degrees. Keep small/realistic.
heading_bias_deg = {
    "A01": 0.0,
    "A02": 0.0,   # surface/reference
    "A03": 4.0,
    "A04": -3.0,
}

absolute_heading = [180, 180, 180, 180]
#absolute_heading = [70, 180, 225, 90]
#absolute_heading = [180, 180, 180, 270]
#absolute_heading = [90, 90, 90, 180]

for i, agent in enumerate(S.agents.values()):
    agent.set_VelocityCmd(body_vels[agent.name], mode="local_velocity")
    agent.set_Heading(absolute_heading[i], mode="step")


# =================================
# Acoustic and ranging
# =================================

ac_handle = activate_Acoustic(
    S,
    c,
    pdr_sender=pdr_sender,
)
Ranging = AcousticRanging(sound_speed=c)


# =================================
# Packet durations
# =================================

guard_time = 2 * ws_radius / c
header_bytes = 12

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
    nav_payload_builder=utils.build_nav_payload,
    min_payload_builder=utils.build_min_payload,
    K_select=K_select,
    policy=policy,
    sender_pdr_prior=pdr_sender,
)

MAC.register_agents(S.agents.values())


# =================================
# Navigation filter
# =================================

Nav = EKFNavFilter(
    writeback=True,
    surface_agents=("A02",),
)

# Nav = FGNavFilter(
#     writeback=True,
#     surface_agents=("A02",),
# )

Nav.register_agents(S.agents.values())

# Store the true/base surface agents. Temporarily promoted agents are added/removed.
base_surface_agents = set(Nav.surface_agents)
utils.initialize_nav_agent_fields(S, Nav)


# =================================
# Sporadic GPS-fix policy
# =================================

gps_fix_enabled = False

gps_fix_duration = 1.0
gps_fix_cooldown = 300.0

gps_fix_until = {}
gps_fix_last = {}

gps_fix_risk_thresh = 1.0
gps_fix_traceP_ref = 2.0
gps_fix_abs_age_ref = 300.0
gps_fix_coop_age_ref = 60.0
gps_fix_nis_ref = 9.0

gps_fix_w_traceP = 0.55
gps_fix_w_abs_age = 0.25
gps_fix_w_coop_age = 0.20
gps_fix_w_nis = 0.00


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
        payload_builder=utils.build_nav_payload,
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


dynamic_scenario_enabled = True
_last_scenario_phase = None
_last_scenario_phase = utils.update_dynamic_scenario(S, MAC, ac_handle, dynamic_scenario_enabled, _last_scenario_phase)


# =================================
# Simulation callback
# =================================

def cycle(nav_logs, coop_logs, coop_update_debug_logs):
    global frame, print_bootstrap, next_local_update_time, next_gps_update_time, _last_scenario_phase

    prev_mac_frame_id = getattr(MAC, "frame_id", None)

    # Physics step
    S.tick()

    # Dynamic scenario update
    _last_scenario_phase  = utils.update_dynamic_scenario(S, MAC, ac_handle, dynamic_scenario_enabled, _last_scenario_phase)

    # MAC step
    delivered = MAC(S)

    # Detect adaptive frame transition
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
                    payload_builder=utils.build_nav_payload,
                    duration=tx_nav_duration,
                )

            frame += 1

    # Ranging extraction
    Ranging(S, delivered)

    # Navigation prediction
    for agent in S.agents.values():
        Nav.predict(agent, S)

    # Cooperative updates
    if ranging:
        Nav.process_cooperative(S, delivered)

    # Local updates
    if S.time + 1e-9 >= next_local_update_time:
        utils.apply_heading_measurement_bias(S,heading_bias,heading_bias_deg)
        for agent in S.agents.values():
            Nav.update_local(agent, S)
        next_local_update_time += local_update_dt

    # GPS/surface XY updates
    if S.time + 1e-9 >= next_gps_update_time:
        if getattr(Nav, "writeback", False):
            Nav._writeback_all(S)

        utils.update_sporadic_gps_fixes(
            sim=S,
            nav=Nav,
            base_surface_agents=base_surface_agents,
            gps_fix_until=gps_fix_until,
            gps_fix_last=gps_fix_last,
            enabled=gps_fix_enabled,
            duration=gps_fix_duration,
            cooldown=gps_fix_cooldown,
            risk_thresh=gps_fix_risk_thresh,
            traceP_ref=gps_fix_traceP_ref,
            abs_age_ref=gps_fix_abs_age_ref,
            coop_age_ref=gps_fix_coop_age_ref,
            nis_ref=gps_fix_nis_ref,
            w_traceP=gps_fix_w_traceP,
            w_abs_age=gps_fix_w_abs_age,
            w_coop_age=gps_fix_w_coop_age,
            w_nis=gps_fix_w_nis,
        )

        for agent in S.agents.values():
            Nav.update_surface_position(agent, S)

        if getattr(Nav, "writeback", False):
            Nav._writeback_all(S)

        next_gps_update_time += gps_update_dt

    # Logging
    utils.log_nav_step(S, Nav, S.agents, nav_logs)
    utils.log_coop_events(S, delivered, Nav, coop_logs)
    utils.log_coop_update_debug(Nav, coop_update_debug_logs)

    # Visualization bookkeeping
    visualizer.last_delivered = delivered

    # Debug print once per adaptive frame
    if new_frame:
        utils.print_adaptive_frame_report(
            sim=S,
            nav=Nav,
            mac=MAC,
            base_surface_agents=base_surface_agents,
        )
        utils.prune_acoustic_ranges(S, max_age=MAC.frame_duration)


def cycle_callback():
    cycle(nav_logs, coop_logs, coop_update_debug_logs)

    if S.time >= SIM_STOP_TIME:
        utils.finalize_and_exit(
            sim=S,
            nav_logs=nav_logs,
            coop_logs=coop_logs,
            coop_update_debug_logs=coop_update_debug_logs,
            log_dir=LOG_DIR,
            log_str=log_str,
            policy=policy
        )


# =================================
# Run visualizer
# =================================

visualizer = Visualizer2D(S, cycle_callback, properties, mac=MAC)

try:
    visualizer.run()
finally:
    utils.save_all_logs(
        nav_logs,
        coop_logs,
        coop_update_debug_logs,
        LOG_DIR,
        log_str,
        policy,
    )
