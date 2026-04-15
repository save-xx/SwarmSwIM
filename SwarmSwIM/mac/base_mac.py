from abc import ABC, abstractmethod
from collections import deque


class Base_MAC(ABC):
    """
    Abstract MAC layer compatible with the SwarmSwIM acoustic channel.

    Responsibilities
    ----------------
    - Maintain per-agent transmission queues
    - Decide when an agent can transmit
    - Call AcousticChannel.send(...)
    - Collect completed receptions from acoustic plugin
    - Maintain protocol statistics
    - Generate frame-level reports once all receivers processed a TX
    """

    def __init__(self, acoustic_handler):

        # acoustic channel handler
        self.acoustic = acoustic_handler

        # per-agent queues
        self.queues = {}
        self.pending_tx = {}

        # statistics
        self.stats = {
            "tx": 0,
            "collisions": 0,
            "denied": 0,
            "rx_success": 0,
            "rx_lost": 0
        }

        # frame-level reporting
        self._frame_buffer = []

        # active transmissions waiting for completion
        self._active_tx = {}

        # number of agents
        self._n_agents = 0

    # ==========================================================
    # Public API
    # ==========================================================

    def register_agents(self, agents):

        agents = list(agents)

        for agent in agents:
            self.queues[agent.name] = deque()
            self.pending_tx[agent.name] = 0.0

        self._n_agents = len(agents)

    def request_tx(self, agent, payload, duration):

        packet = {
            "agent": agent,
            "payload": payload,
            "duration": duration
        }

        self.queues[agent.name].append(packet)

    def get_frame_report(self):

        report = self._frame_buffer.copy()
        self._frame_buffer.clear()

        return report

    # ==========================================================
    # MAC step
    # ==========================================================

    def __call__(self, sim):

        # scheduling decision
        self._schedule(sim)

        # resolve acoustic propagation
        delivered = self.acoustic._event_to_save

        # update statistics + reports
        self._update_rx_stats(delivered)

        # protocol-specific hooks
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
    # Transmission helper
    # ==========================================================

    def _attempt_tx(self, sim, packet):

        agent = packet["agent"]
        payload = packet["payload"]
        duration = packet["duration"]

        success, status = self.acoustic.send(agent, payload, duration)

        if success:

            self.stats["tx"] += 1

            sender = agent.name

            # create transmission tracking entry
            self._active_tx[sender] = {
                "sender": sender,
                "tx_time": sim.time,
                "rx_success": [],
                "rx_lost": [],
                "processed": 0
            }

        else:

            if "Collision avoidance" in status:
                self.stats["denied"] += 1
            else:
                self.stats["collisions"] += 1

        return success

    # ==========================================================
    # Reception statistics + frame reports
    # ==========================================================

    def _update_rx_stats(self, delivered):

        for receiver, msg in delivered.items():

            if msg is None:
                continue

            sender = msg.sender

            tx = self._active_tx.get(sender)

            if tx is None:
                continue

            tx["processed"] += 1

            if msg.intact:

                self.stats["rx_success"] += 1
                tx["rx_success"].append(receiver)

            else:

                self.stats["rx_lost"] += 1
                tx["rx_lost"].append(receiver)

            # if all receivers processed → close report
            if tx["processed"] == self._n_agents - 1:

                self._frame_buffer.append({
                    "sender": sender,
                    "rx_success": tx["rx_success"].copy(),
                    "rx_lost": tx["rx_lost"].copy(),
                    "tx_time": tx["tx_time"]
                })

                del self._active_tx[sender]