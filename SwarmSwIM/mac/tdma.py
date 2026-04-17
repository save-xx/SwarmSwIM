from .base_mac import Base_MAC


class TDMA_MAC(Base_MAC):
    """
    Framed TDMA MAC compatible with the new SwarmSwIM acoustic channel.

    Assumptions
    ----------
    - Global synchronization
    - One slot per agent
    - No retransmissions
    """

    def __init__(self, acoustic_handler,
                 slot_duration,
                 frame_duration,
                 guard_time=0.0):

        super().__init__(acoustic_handler)

        self.slot_duration = float(slot_duration)
        self.frame_duration = float(frame_duration)
        self.guard_time = float(guard_time)

        self.agents_order = []

    # ----------------------------------------------------------

    def register_agents(self, agents):
        super().register_agents(agents)
        self.agents_order = [a.name for a in agents]

    # ----------------------------------------------------------

    def _schedule(self, sim):
        """
        Allow transmission only for the agent owning the current slot.
        """

        if not self.agents_order:
            return

        t = sim.time

        # time inside frame
        t_frame = t % self.frame_duration

        # slot index
        slot_idx = int(t_frame // self.slot_duration)

        if slot_idx >= len(self.agents_order):
            return

        agent_name = self.agents_order[slot_idx]

        slot_start = slot_idx * self.slot_duration
        slot_end = slot_start + self.slot_duration - self.guard_time

        tol = 0.0
        # ensure we are inside the usable part of the slot
        if not (slot_start+tol <= t_frame < slot_end+tol):
            return

        # if the agent has queued packets, transmit one
        if self.queues[agent_name]:

            packet = self.queues[agent_name].popleft()

            self._attempt_tx(sim, packet)