from abc import ABC, abstractmethod
from collections import deque


class Base_MAC(ABC):
    """
    Abstract MAC layer.

    Responsibilities:
    - Maintain per-agent transmission queues
    - Decide when an agent can transmit
    - Call AcousticChannel.send(...)
    - Process delivered messages
    - Maintain protocol-level statistics
    """

    def __init__(self, acoustic_channel):
        self.channel = acoustic_channel

        self.queues = {}
        self.pending_tx = {}

        # Full MAC-level stats
        self.stats = {
            "tx": 0,            # transmission attempts
            "collisions": 0,    # collided transmissions
            "denied": 0,        # denied transmissions
            "rx_success": 0,    # successful receptions
            "rx_lost": 0        # natural packet loss
        }

    # ==========================================================
    # Public API
    # ==========================================================

    def register_agents(self, agents):
        for agent in agents:
            self.queues[agent.name] = deque()
            self.pending_tx[agent.name] = 0.0

    def request_tx(self, agent, payload, duration):
        packet = {
            "agent": agent,
            "payload": payload,
            "duration": duration
        }
        self.queues[agent.name].append(packet)

    def get_frame_report(self):
        report = getattr(self, "_frame_buffer", [])
        self._frame_buffer = []
        return report

    def __call__(self, sim):

        #scheduling decision
        self._schedule(sim)

        #physical propagation - and tx/rx reports
        delivered, channel_report, frame_report_tick = self.channel(sim)

        #update MAC stats from channel report
        self._update_stats(channel_report)

        # accumulate per-frame report (so we can print at lower frequency)
        if not hasattr(self, "_frame_buffer"):
            self._frame_buffer = []

        self._frame_buffer.extend(frame_report_tick)

        #protocol-level processing (ACK/backoff etc.)
        self._process_delivered(sim, delivered)

        return delivered

    # ==========================================================
    # Internal hooks
    # ==========================================================

    @abstractmethod
    def _schedule(self, sim):
        pass

    def _process_delivered(self, sim, delivered):
        pass

    # ==========================================================
    # Utility
    # ==========================================================

    def _attempt_tx(self, sim, packet):

        agent = packet["agent"]
        app_payload = packet["payload"]
        duration = packet["duration"]

        frame = {
            "id": agent.name,
            "tx_time": sim.time,
            "duration": duration,
            "data": app_payload
        }

        result = self.channel.send(agent, sim, duration, frame)

        # Transmission-level stats
        if result == "sent":
            self.stats["tx"] += 1
        elif result == "collision":
            self.stats["tx"] += 1
            self.stats["collisions"] += 1
        elif result == "denied":
            self.stats["denied"] += 1

        return result

    def _update_stats(self, channel_report):
        """
        channel_report contains reception outcomes
        from AcousticChannel.
        """

        self.stats["rx_success"] += channel_report["rx_success"]
        self.stats["rx_lost"] += channel_report["rx_lost"]