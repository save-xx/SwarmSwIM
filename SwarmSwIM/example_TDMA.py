from SwarmSwIM import Simulator
from SwarmSwIM import activate_Acoustic, activate_Currents, save_bag
from SwarmSwIM import Visualizer2D

from mac.tdma import TDMA_MAC
from sensors.acoustic_ranging import AcousticRanging

import json


# =================================
# Simulation parameters
# =================================

ws_radius = 200

fps_physics = 30
fps_render  = 30
steps_per_frame = fps_physics // fps_render
# acoustic channel 
bps = 450
PDR = 1.0
c = 1500


payload_template = {
    "id": 0,
    "tx_time": 0.0,
    "data": {"pos": [0.0, 0.0, 0.0]}
}


# =================================
# Simulator
# =================================

S = Simulator(1 / fps_physics)

#save_bag(S)
activate_Currents(S)

# Activate acoustic channel
ac_handle = activate_Acoustic(S,c,PDR)

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

counter = 0

def cycle():

    global counter

    counter += 1

    # =================================
    # TDMA scheduling
    # =================================

    if counter % frame_steps == 0:

        for agent in S.agents.values():

            MAC.request_tx(
                agent,
                payload={"pos": agent.pos.tolist()},
                duration=tx_duration
            )

    # =================================
    # Physics step (acoustic plugin runs here)
    # =================================

    events = S.tick()

    # =================================
    # MAC step (read acoustic events)
    # =================================

    delivered = MAC(S)

    # =================================
    # Ranging extraction
    # =================================

    Ranging(S, delivered)

    # Update visualization
    visualizer.last_delivered = delivered

    # =================================
    # Print once per TDMA frame
    # =================================

    if counter % frame_steps == 0:

        print(f"\n{'='*20} TDMA Frame @ t={S.time:6.2f}s {'='*20}")

        # -------------------------------------------------
        # Positions
        # -------------------------------------------------
        print("\n--- Positions ---")

        for agent in S.agents.values():
            print(
                f"{agent.name:>3} | "
                f"x={agent.pos[0]:8.3f}  "
                f"y={agent.pos[1]:8.3f}  "
                f"z={agent.pos[2]:6.3f}"
            )

        # -------------------------------------------------
        # Acoustic Ranging
        # -------------------------------------------------
        print("\n--- Acoustic Ranges ---")

        for agent in S.agents.values():

            if not hasattr(agent, "AcousticRange") or not agent.AcousticRange:
                continue

            print(f"\n{agent.name} receives:")

            for other, meas in agent.AcousticRange.items():

                age = S.time - meas["t_meas"]

                print(
                    f"  {other:>3} | "
                    f"d={meas['range']:7.2f} m  "
                    f"tof={meas['tof']:7.4f} s  "
                    f"dop={meas['doppler']:7.3f} m/s  "
                    f"age={age:6.3f} s"
                )

        # -------------------------------------------------
        # MAC statistics
        # -------------------------------------------------
        print("\n--- MAC stats (TDMA) ---")

        print(
            f"TX={MAC.stats['tx']:3d} | "
            f"RX_OK={MAC.stats['rx_success']:3d} | "
            f"RX_LOST={MAC.stats['rx_lost']:3d} | "
            f"COLL={MAC.stats['collisions']:3d} | "
            f"DENIED={MAC.stats['denied']:3d}"
        )

        # -------------------------------------------------
        # Frame report
        # -------------------------------------------------
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

        # -------------------------------------------------
        # Remove old measurements
        # -------------------------------------------------
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

visualizer = Visualizer2D(S, cycle, properties, mac=MAC)
visualizer.run()
