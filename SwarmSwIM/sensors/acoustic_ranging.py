import numpy as np


class AcousticRanging:
    """
    Event-based one-way time-of-flight ranging.

    A range measurement is created ONLY when a packet is received.
    It estimates:
        tof = t_rx - t_tx
        range = c * tof
    assuming no clock drift/offset (simulation assumption).
    """

    def __init__(self, sound_speed=1500.0, rnd=None):
        self.c = float(sound_speed)
        self.rnd = rnd if rnd is not None else np.random.default_rng()

    def emulate_error(self, value, error):
        """error = [bias, std]"""
        return float(value) + float(error[0]) + self.rnd.normal(scale=float(error[1]))

    def __call__(self, Simulator, delivered):

        t_rx = float(Simulator.time)

        agents_by_name = {a.name: a for a in Simulator.agents}

        for rx_name, (frame, _) in delivered.items():

            rx = agents_by_name.get(rx_name, None)
            if rx is None:
                continue

            rx.AcousticRange = {}

            if not isinstance(frame, dict):
                continue

            sender_id = frame.get("id", None)
            tx_time = frame.get("tx_time", None)
            
            if sender_id is None or tx_time is None:
                continue
            
            tof = t_rx - tx_time
            if tof < 0.0:
                continue
            
            range_true = self.c * tof

            # Apply range noise (bias + Gaussian)
            if hasattr(rx, "sensors") and ("e_ac_range" in rx.sensors):
                bias, std = rx.sensors["e_ac_range"]
                range_meas = range_true + bias + self.rnd.normal(scale=std)
            else:
                range_meas = range_true

            rx.AcousticRange[sender_id] = {
                "range": float(range_meas),
                "tof": float(range_meas) / self.c,
                "t_rx": t_rx,
                "t_tx": float(tx_time),
            }