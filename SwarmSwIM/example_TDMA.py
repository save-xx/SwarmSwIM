from .agent_class import Agent
from .sim_class import Simulator
from .sensors.visual_detection import CNNDetection
from .sensors.acoustic_comm import AcousticChannel
from .animator2D import Plotter
from .mac.tdma import TDMA_MAC
from .sensors.acoustic_ranging import AcousticRanging

import matplotlib.pyplot as plt
import json


# ===============================
# Simulation parameters
# ===============================

ws_radius = 200          # workspace radius (m)
c = 1500                 # sound speed (m/s)

fps_physics = 500        # physics integration rate (Hz)
fps_render  = 30         # rendering rate (Hz)
steps_per_frame = fps_physics // fps_render

bps = 450                # acoustic bitrate (bps)

PDR = 0.5 #Packet Delivery Ratio (0-1), i.e., 0.3 is 30%

payload_template = {
    "id": 0,
    "tx_time": 0.0,
    "data": {"pos": [0.0, 0.0, 0.0]}
}


# ===============================
# Initialize simulator
# ===============================

S = Simulator(1 / fps_physics)
Ranging = AcousticRanging(sound_speed=c)

circle = plt.Circle((0, 0), ws_radius, color='g', fill=False, alpha=0.5)

for agent in S.agents:
    agent.cmd_forces = 1.0
    agent.cmd_local_vel = 0.1


# ===============================
# Acoustic + TDMA parameters
# ===============================

guard_time = 2 * ws_radius / c

payload_bytes = json.dumps(payload_template).encode("utf-8")
header_bytes = 8
total_bits = (len(payload_bytes) + header_bytes) * 8

tx_duration = total_bits / bps
slot_duration = tx_duration + guard_time
frame_duration = slot_duration * len(S.agents)

Acoustic = AcousticChannel(pdr=PDR)
MAC = TDMA_MAC(
    Acoustic,
    slot_duration=slot_duration,
    frame_duration=frame_duration,
    guard_time=guard_time
)

MAC.register_agents(S.agents)

Animation = Plotter(S, fps_render, SIZE=ws_radius * 1.2, artistics=[circle])

# ===============================
# Animation callback
# ===============================

counter = 0

def animation_callback():
    global counter

    # Run multiple physics steps per render frame
    for _ in range(steps_per_frame):

        counter += 1

        # Physics integration
        S.tick()

        # TDMA scheduling
        if counter % int(frame_duration * fps_physics) == 0:
            for agent in S.agents:
                MAC.request_tx(
                    agent,
                    payload={"pos": agent.pos.tolist()},
                    duration=tx_duration
                )

        # Channel propagation
        delivered = MAC(S)

        # Ranging
        Ranging(S, delivered)

        # Print once per TDMA time frame
        if counter % int(frame_duration * fps_physics) == 0:
            print(f"\n===== TDMA Frame @ t={S.time:.2f}s =====")
            print('--- Positions ---')
            for agent in S.agents:
                print(f'{agent.name}: '
                    f'x={agent.pos[0]:.3f}, '
                    f'y={agent.pos[1]:.3f}, '
                    f'z={agent.pos[2]:.3f}, '
                    f'psi={agent.psi:.1f}')
                
            print('--- Acoustic Ranges ---')
            for agent in S.agents: 
                if hasattr(agent, "AcousticRange") and agent.AcousticRange: 
                    print(agent.name, agent.AcousticRange) 
                    for other, meas in agent.AcousticRange.items(): 
                        print(f' {other}: d={meas["range"]:.2f} m, tof={meas["tof"]:.4f} s')

            print('--- MAC stats (TDMA)/Previous Frame ---')
            print({
                "tx": MAC.stats["tx"],
                "rx_success": MAC.stats["rx_success"],
                "rx_lost": MAC.stats["rx_lost"],
                "collisions": MAC.stats["collisions"],
                "denied": MAC.stats["denied"]
            })
            frame_report = MAC.get_frame_report()
            for tx in frame_report:
                print(f"TX {tx['sender']} -> "
                    f"OK: {tx['rx_success']} "
                    f"LOST: {tx['rx_lost']}")
            
            print()


# ===============================
# Run animation
# ===============================

Animation.update_plot(callback=animation_callback)