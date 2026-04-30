from .base_mac import Base_MAC


class Adaptive_TDMA_MAC(Base_MAC):
    """
    Adaptive TDMA with:
    - fixed slot order
    - variable slot duration
    - two payload modes: min / nav
    - fixed leader
    - no agreement yet

    Current policy:
    - leader ranks agents by q_i = 1 / trace(P_pos)
    - top-K agents send NAV
    - others send MIN
    - frame duration changes accordingly
    """

    def __init__(
        self,
        acoustic_handler,
        nav_duration,
        min_duration,
        guard_time,
        leader_id,
        nav_payload_builder,
        min_payload_builder,
        K_select=2,
    ):
        super().__init__(acoustic_handler)

        self.nav_duration = float(nav_duration)
        self.min_duration = float(min_duration)
        self.guard_time = float(guard_time)

        self.leader_id = str(leader_id)
        self.nav_payload_builder = nav_payload_builder
        self.min_payload_builder = min_payload_builder
        self.K_select = int(K_select)

        self.agents_order = []

        self.frame_id = -1
        self.frame_start_time = 0.0
        self.frame_duration = 0.0

        self.active_modes = {}
        self.slot_table = []

        self.q_table = {}

    # ----------------------------------------------------------

    def register_agents(self, agents):
        super().register_agents(agents)
        self.agents_order = [a.name for a in agents]

        # bootstrap: everyone sends min in first frame
        #self.active_modes = {name: "min" for name in self.agents_order}
        self.active_modes = {name: "nav" for name in self.agents_order}
        self._rebuild_slot_table()
        self.frame_id = 0
        self.frame_start_time = 0.0

    # ----------------------------------------------------------

    def _schedule(self, sim):
        if not self.agents_order:
            return

        t = float(sim.time)

        while t >= self.frame_start_time + self.frame_duration - 1e-12:
            self.frame_start_time += self.frame_duration
            self._start_new_frame(sim)

        t_frame = t - self.frame_start_time

        slot = self._find_active_slot(t_frame)
        if slot is None:
            return

        agent_name = slot["agent"]
        mode = slot["mode"]

        if not self.queues[agent_name]:
            return

        packet = self.queues[agent_name].popleft()

        if mode == "nav":
            packet["payload_builder"] = self._build_nav_plus_min_wrapper(packet["agent"], sim)
            packet["duration"] = self.min_duration + self.nav_duration
        else:
            packet["payload_builder"] = self.min_payload_builder
            packet["duration"] = self.min_duration

        self._attempt_tx(sim, packet)

    # ----------------------------------------------------------

    def _start_new_frame(self, sim):
        self.frame_id += 1

        self._update_q_table(sim)
        self.active_modes = self._compute_modes_for_frame(sim)
        self._rebuild_slot_table()

    # ----------------------------------------------------------

    def _update_q_table(self, sim):
        for agent in sim.agents.values():
            nav_info = getattr(agent, "nav_info", None)
            if not nav_info:
                continue

            traceP = nav_info.get("traceP_pos", None)
            if traceP is None:
                continue

            try:
                traceP = float(traceP)
            except Exception:
                continue

            if traceP > 0.0:
                self.q_table[agent.name] = 1.0 / traceP

    # ----------------------------------------------------------

    def _compute_modes_for_frame(self, sim):
        modes = {name: "min" for name in self.agents_order}

        ranked = sorted(
            self.q_table.items(),
            key=lambda item: item[1],
            reverse=True,
        )

        selected = [name for name, _ in ranked[:self.K_select]]
        selected = [name for name in selected if name in modes]

        for name in selected:
            modes[name] = "nav"

        return modes

    # ----------------------------------------------------------

    def _rebuild_slot_table(self):
        self.slot_table = []
        t0 = 0.0

        for name in self.agents_order:
            mode = self.active_modes.get(name, "min")
            if mode == "nav":
                tx_dur = self.min_duration + self.nav_duration  
            else:
                tx_dur = self.min_duration

            self.slot_table.append({
                "agent": name,
                "mode": mode,
                "start": t0,
                "tx_end": t0 + tx_dur,
                "slot_end": t0 + tx_dur + self.guard_time,
            })

            t0 += tx_dur + self.guard_time

        self.frame_duration = t0

    # ----------------------------------------------------------

    def _find_active_slot(self, t_frame):
        for slot in self.slot_table:
            if slot["start"] <= t_frame < slot["tx_end"]:
                return slot
        return None
    
    def _build_nav_plus_min_wrapper(self, agent, sim):
        def builder(agent_obj, tx_time):
            min_payload = self.min_payload_builder(agent_obj, tx_time)
            nav_payload = self.nav_payload_builder(agent_obj, tx_time)

            payload = dict(min_payload)
            payload.update(nav_payload)
            payload["type"] = "nav"
            return payload

        return builder