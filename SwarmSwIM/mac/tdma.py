import math
from .base_mac import Base_MAC


class TDMA_MAC(Base_MAC):
    """
    Framed TDMA MAC.

    Assumptions:
    - Global time synchronization
    - One fixed slot per agent
    - No retransmissions
    """

    def __init__(self, acoustic_channel,
                 slot_duration, frame_duration,
                 guard_time=0.0):
        super().__init__(acoustic_channel)

        self.slot_duration = slot_duration
        self.guard_time = guard_time

        self.agents_order = []
        self.frame_duration = frame_duration

    # ---------------------------------

    def register_agents(self, agents):
        super().register_agents(agents)
        self.agents_order = [a.name for a in agents]

    # ---------------------------------

    def _schedule(self, sim):
        """
        Check current slot owner and allow transmission
        only if inside its slot.
        """

        if self.frame_duration is None:
            return

        t = sim.time

        # Position inside current frame
        t_frame = t % self.frame_duration

        # Determine slot index
        slot_idx = int(t_frame // self.slot_duration)

        if slot_idx >= len(self.agents_order):
            return

        agent_name = self.agents_order[slot_idx]

        # Check if inside guard time
        slot_start = slot_idx * self.slot_duration
        slot_end = slot_start + self.slot_duration - self.guard_time

        if not (slot_start <= t_frame < slot_end):
            return

        # If agent has queued packet → transmit one
        if self.queues[agent_name]:
            packet = self.queues[agent_name].popleft()
            self._attempt_tx(sim, packet)