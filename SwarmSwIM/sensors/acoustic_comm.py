import numpy as np
import time

class AcousticChannel:
    """
    Acoustic broadcast channel model.

    - Finite propagation speed
    - Finite packet duration (channel occupancy)
    - Leading-edge reception
    - Optional collision/denial
    - Natural packet loss via PDR

    Returns:
        delivered: dict of received packets
        channel_report: {"rx_success": int, "rx_lost": int}
    """

    def __init__(self, pdr=1.0, sound_speed=1500.0,
                 max_range=2000.0, computational_delay=0.05):

        self.C = float(sound_speed)
        self.MAX_RANGE = float(max_range)

        self.COMPUTATIONAL_DELAY = float(computational_delay)
        self.BUFF_RADIUS = self.C * self.COMPUTATIONAL_DELAY

        self.PDR = float(pdr)
        self.rnd = np.random.default_rng()

        self.channel_status = []

    # ==========================================================
    # Transmission
    # ==========================================================

    def send(self, agent, sim, duration, frame=None):

        duration = float(duration)
        # MAC defines tx_time
        t0 = frame["tx_time"]

        status = "active"
        idx_collision = None

        # Check interaction with ongoing transmissions
        for i, event in enumerate(self.channel_status):

            if status == "denied":
                break

            for circle in event["circles"]:

                d = np.linalg.norm(circle["center"] - agent.pos)

                # Behind trailing edge → no effect
                if d < circle["radii"][0]:
                    continue

                # Too close to active wavefront → deny
                if d < circle["radii"][1] - self.BUFF_RADIUS:
                    status = "denied"
                    break

                # Otherwise collision
                status = "collision"
                idx_collision = i

        if status == "denied":
            return "denied"

        new_circle = {
            "radii": [0.0, 0.0],           # [trailing, leading]
            "times": [t0, t0 + duration],  # [start, end]
            "center": agent.pos.copy(),
        }

        if status == "collision":

            self.channel_status[idx_collision]["circles"].append(new_circle)
            self.channel_status[idx_collision]["status"] = "collision"
            self.channel_status[idx_collision]["frame"] = None
            self.channel_status[idx_collision]["sender"] = "Failed"
            self.channel_status[idx_collision]["processed"].append(agent)

            return "collision"

        # Active transmission
        event = {
            "circles": [new_circle],
            "status": "active",
            "frame": frame,
            "sender": agent.name,
            "tx_time": t0,
            "duration": duration,
            "processed": [agent],   # use names (stable), not agent objects
            "rx_success": [],
            "rx_lost": [],
            "reported": False
            }

        self.channel_status.append(event)
        return "sent"

    # ==========================================================
    # Propagation + Reception
    # ==========================================================

    def __call__(self, sim):

        t = float(sim.time)
        delivered = {}

        rx_success = 0
        rx_lost = 0

        frame_report = []

        for event in self.channel_status:

            # Update wavefronts
            for circle in event["circles"]:
                circle["radii"][1] = max(0.0, (t - circle["times"][0]) * self.C)
                circle["radii"][0] = max(0.0, (t - circle["times"][1]) * self.C)

            agents2check = [a for a in sim.agents if a not in event["processed"]]

            for rx in agents2check:

                received = True

                for circle in event["circles"]:
                    d = np.linalg.norm(rx.pos - circle["center"])

                    # Leading-edge reception
                    if d > circle["radii"][1]:
                        received = False
                        break

                if received:
                    # now the deliver event depend also on natural packet loss
                    event["processed"].append(rx)
                    # Simulate natural packet loss and log successful or failed reception
                    if self.rnd.random() <= self.PDR:

                        delivered[rx.name] = [event["frame"], event["sender"]]
                        event["rx_success"].append(rx.name)
                        rx_success += 1                   

                    else:
                        event["rx_lost"].append(rx.name)
                        rx_lost += 1                    

                # Update frame_report with info for this specifc tranmission 
                # (so we can see who receiv who not), after all the reception for this transmission are processed
                if len(event["processed"]) == len(sim.agents):

                    frame_report.append({
                        "sender": event["sender"],
                        "rx_success": event["rx_success"].copy(),
                        "rx_lost": event["rx_lost"].copy(),
                        "tx_time": event["tx_time"],
                    })
                    event["reported"] = True

        # Cleanup expired events
        for event in self.channel_status:
            event["circles"] = [
                c for c in event["circles"]
                if c["radii"][0] <= self.MAX_RANGE
            ]

        self.channel_status = [
            e for e in self.channel_status
            if e["circles"]
        ]

        channel_report = {
            "rx_success": rx_success,
            "rx_lost": rx_lost
        }

        

        return delivered, channel_report, frame_report