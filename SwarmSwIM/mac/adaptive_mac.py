from .base_mac import Base_MAC


class Adaptive_TDMA_MAC(Base_MAC):
    """
    Adaptive TDMA MAC with:
    - fixed leader
    - schedule proposal for next frame
    - agreement gate before activation
    - fallback to baseline TDMA otherwise

    Behavior
    --------
    Frame k:
        - leader proposes schedule for frame k+1
        - agents exchange consensus payloads
        - a network-wide agreement check is performed

    Frame k+1:
        - apply adaptive schedule only if agreement was reached for k+1
        - otherwise use baseline TDMA

    Notes
    -----
    This is not full distributed consensus. It is:
    - fixed leader
    - simplified agreement on the received leader proposal
    - safe fallback under packet loss

    In this minimal implementation:
    - TDMA slot ownership remains fixed
    - adaptive scheduling changes payload type only
    """

    def __init__(
        self,
        acoustic_handler,
        slot_duration,
        frame_duration,
        leader_id,
        guard_time=0.0,
        K_select=2,
    ):
        super().__init__(acoustic_handler)

        self.slot_duration = float(slot_duration)
        self.frame_duration = float(frame_duration)
        self.guard_time = float(guard_time)

        self.leader_id = str(leader_id)
        self.K_select = int(K_select)

        self.agents_order = []

        # frame bookkeeping
        self.frame_id = -1

        # current schedule actually used in THIS frame
        self.active_schedule = None
        self.active_schedule_frame = None

        # proposal being disseminated for NEXT frame
        self.proposed_schedule = None
        self.proposed_schedule_frame = None

        # agreement result for the NEXT frame
        self.agreement_ok = False
        self.agreement_frame = None

        # last agreed leader proposal
        self.received_leader_proposal = None

        # latest nav quality values known locally
        self.q_table = {}

    # ----------------------------------------------------------

    def register_agents(self, agents):
        super().register_agents(agents)
        self.agents_order = [a.name for a in agents]

    # ----------------------------------------------------------

    def _schedule(self, sim):
        if not self.agents_order:
            return

        t = float(sim.time)
        frame_id = int(t // self.frame_duration)

        if frame_id != self.frame_id:
            self._start_new_frame(sim, frame_id)

        t_frame = t % self.frame_duration
        slot_idx = int(t_frame // self.slot_duration)

        if slot_idx >= len(self.agents_order):
            return

        slot_start = slot_idx * self.slot_duration
        slot_end = slot_start + self.slot_duration - self.guard_time

        if not (slot_start <= t_frame < slot_end):
            return

        agent_name = self._get_slot_owner(slot_idx)

        if agent_name is None:
            return

        if not self.queues[agent_name]:
            return

        packet = self.queues[agent_name].popleft()
        agent = packet["agent"]

        # In adaptive mode:
        # - scheduled agents send NAV
        # - others send CONSENSUS
        #
        # In fallback mode:
        # - everyone sends CONSENSUS in its baseline TDMA slot
        if self._using_adaptive_schedule_this_frame():
            if agent_name in self.active_schedule:
                pass
            else:
                packet["payload_builder"] = self._build_consensus_wrapper(agent, sim)
        else:
            packet["payload_builder"] = self._build_consensus_wrapper(agent, sim)

        self._attempt_tx(sim, packet)

    # ----------------------------------------------------------

    def _process_delivered(self, sim, delivered):
        """
        Process delivered consensus messages.

        Agreement rule for the current global MAC implementation:
        - accept schedule for frame k+1 only if a valid leader proposal
          is received intact by all non-leader agents
        """
        expected_next_frame = self.frame_id + 1
        valid_receivers = set()
        candidate_proposal = None

        for receiver_name, msg in delivered.items():
            if msg is None or not getattr(msg, "intact", False):
                continue

            payload = msg.payload
            if payload is None:
                continue

            if payload.get("type") != "consensus":
                continue

            sender = payload.get("id", None)
            if sender is None:
                continue

            # Store q information from all received consensus payloads
            q_val = payload.get("q", None)
            if q_val is not None:
                try:
                    q_val = float(q_val)
                    if q_val >= 0.0:
                        self.q_table[sender] = q_val
                except Exception:
                    pass

            # Only the leader proposal is eligible for agreement
            if payload.get("leader_id") != self.leader_id:
                continue
            if sender != self.leader_id:
                continue

            proposal_frame = payload.get("proposal_frame", None)
            if proposal_frame is None:
                continue

            try:
                proposal_frame = int(proposal_frame)
            except Exception:
                continue

            if proposal_frame != expected_next_frame:
                continue

            schedule = payload.get("schedule", None)
            if not isinstance(schedule, list):
                continue

            valid_receivers.add(receiver_name)

            if candidate_proposal is None:
                candidate_proposal = {
                    "proposal_frame": proposal_frame,
                    "leader_id": self.leader_id,
                    "schedule": list(schedule),
                }

        # Network-wide agreement:
        # all non-leader agents must have received the valid leader proposal
        expected_receivers = {name for name in self.agents_order if name != self.leader_id}

        if candidate_proposal is not None and valid_receivers == expected_receivers:
            self.received_leader_proposal = candidate_proposal
            self.agreement_ok = True
            self.agreement_frame = expected_next_frame
        else:
            self.received_leader_proposal = None
            self.agreement_ok = False
            self.agreement_frame = None

    # ----------------------------------------------------------

    def _start_new_frame(self, sim, new_frame_id):
        """
        Transition to a new frame.

        Order:
        1. Activate schedule for this frame only if agreement was reached earlier
        2. Refresh local q table from local nav_info
        3. Leader computes a proposal for next frame
        """
        self.frame_id = int(new_frame_id)

        # Step 1: activate schedule only if previously agreed
        if (
            self.agreement_ok
            and self.agreement_frame == self.frame_id
            and self.received_leader_proposal is not None
        ):
            self.active_schedule = list(self.received_leader_proposal["schedule"])
            self.active_schedule_frame = self.frame_id
        else:
            self.active_schedule = None
            self.active_schedule_frame = None

        # Clear one-shot agreement latch after frame transition
        self.agreement_ok = False
        self.agreement_frame = None
        self.received_leader_proposal = None

        # Step 2: update local q table from currently available nav info
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

        # Step 3: leader computes proposal for next frame
        next_frame = self.frame_id + 1
        self.proposed_schedule = self._compute_schedule_for_frame(sim, next_frame)
        self.proposed_schedule_frame = next_frame

    # ----------------------------------------------------------

    def _compute_schedule_for_frame(self, sim, proposal_frame):
        """
        Minimal policy:
        rank agents by q_i and select top-K.
        """
        ranked = sorted(
            self.q_table.items(),
            key=lambda item: item[1],
            reverse=True,
        )

        selected = [name for name, _ in ranked[:self.K_select]]

        # Keep only known registered agents
        selected = [name for name in selected if name in self.queues]

        return selected

    # ----------------------------------------------------------

    def _get_slot_owner(self, slot_idx):
        """
        Fixed TDMA slot ownership.

        Adaptive scheduling does not reorder slots.
        It only changes whether the slot carries:
        - NAV payload
        - MIN / consensus payload
        """
        if slot_idx >= len(self.agents_order):
            return None

        return self.agents_order[slot_idx]

    # ----------------------------------------------------------

    def _using_adaptive_schedule_this_frame(self):
        return (
            self.active_schedule is not None
            and self.active_schedule_frame == self.frame_id
        )

    # ----------------------------------------------------------

    def _build_consensus_wrapper(self, agent, sim):
        """
        Consensus payload always carries:
        - local q_i
        - leader proposal for next frame (only meaningful when sent by leader)
        """
        def builder(agent_obj, tx_time):
            nav_info = getattr(agent_obj, "nav_info", {}) or {}
            traceP = float(nav_info.get("traceP_pos", 1.0))
            q_val = 1.0 / traceP if traceP > 0.0 else 0.0

            is_leader = (agent_obj.name == self.leader_id)

            payload = {
                "type": "consensus",
                "id": agent_obj.name,
                "tx_time": tx_time,
                "q": q_val,
                "leader_id": self.leader_id,
                "proposal_frame": self.proposed_schedule_frame,
                "schedule": self.proposed_schedule if is_leader else None,
            }
            return payload

        return builder