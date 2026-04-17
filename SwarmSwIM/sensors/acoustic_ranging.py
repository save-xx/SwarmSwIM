import numpy as np


class AcousticRanging:
    """
    Extract acoustic ranging measurements from the SwarmSwIM acoustic channel.

    The channel already computes:
        - ToD_exact
        - ToA_exact
        - ping_range
        - perfect_range
        - doppler_velocity

    This module simply converts those into a clean per-agent measurement
    structure usable by navigation filters.
    """

    def __init__(self, sound_speed=1500.0):
        self.c = float(sound_speed)

    def __call__(self, simulation, delivered_msgs):

        for receiver_name, msg in delivered_msgs.items():

            receiver = simulation.agents[receiver_name]

            # initialize storage
            if not hasattr(receiver, "AcousticRange"):
                receiver.AcousticRange = {}

            # skip corrupted packets
            if not msg.intact:
                continue

            sender = msg.sender

            # compute ToF
            #tof = msg.ToA_exact - msg.ToD_exact - msg.duration
 
            t_tx = float(msg.ToD_exact)
            tof = float(msg.perfect_range / self.c)

            t_range = t_tx + tof
            t_packet_done = float(msg.ToA_exact)

            receiver.AcousticRange[sender] = {
                "range": msg.ping_range,
                "perfect_range": msg.perfect_range,
                "tof": tof,

                "t_tx": t_tx,

                # first-arrival / ranging time
                "t_rx": t_range,
                "t_meas": t_range,

                # optional: keep packet completion time too
                "t_packet_done": t_packet_done,

                "doppler": msg.doppler_velocity,
            }