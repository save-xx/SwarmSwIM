import numpy as np
from collections import defaultdict, deque
import xml.etree.ElementTree as ET
import logging
import copy


logger = logging.getLogger(__name__)
DEFAULT_LENGTH = 800



class HistoryShortMemory:
    def __init__(self, sim):
        self.sim = sim
        # load length
        self.set_history_length()
        self.initiate_agents()
        print('self.HISTORY_LENGTH',self.HISTORY_LENGTH)


    def initiate_agents(self) -> None:
        """Initate assocuiated attribute for each agent."""
        # Initiate shared time axis memory
        n = self.HISTORY_LENGTH
        self.time_axis= deque( #(andrea)
                [
                    self.sim.time - k * self.sim.Dt for k in reversed(range(n))
                ], maxlen=n
            )
        #self.time_axis = deque([self.sim.time], maxlen=n)
        # for each agent initiate individuel deque memory
        for _, agent in self.sim.agents.items():
            new_deque = deque([agent.pos.copy() for _ in range(n)], maxlen=n)
            setattr(agent, "memory", new_deque)


    def set_history_length(self):
        """Load history length from simulation file, fallback to default if invalid."""
        try:
            tree = ET.parse(self.sim._simulation_filepath)
            root = tree.getroot()
            node = root.find("history_length")
            if node is not None:#(andrea)
                self.HISTORY_LENGTH = int(node.text)
            else:
                self.HISTORY_LENGTH = DEFAULT_LENGTH
            

        except Exception as e:
            self.HISTORY_LENGTH = DEFAULT_LENGTH
            logger.info(
                f"Could not read/convert history_length, revert to default: {DEFAULT_LENGTH}"
            )
            logger.debug(f"Exception: {e}")
        


    def __call__(self):
        """Keep a record of all positions in the last HISTORY_LENGTH steps."""
        # update time_axis
        self.time_axis.append(self.sim.time)

        # update history
        for name, agent in self.sim.agents.items():
            # make a copy of the current position
            position = agent.pos.copy()
            agent.memory.append(position)


    def recall_position(self, t_req, name):
        """Recall the position of an agent at a certain time.""" 
        # check agent existence
        if name not in self.sim.agents:
            raise KeyError(f"Agent name {name} not found in the simulation")
        agent = self.sim.agents[name]
        
        if not self.time_axis:  # empty history
            raise RuntimeError("History is empty, cannot recall position")

        # check if requesting time is future time compared to time_axis
        if t_req > self.time_axis[-1]:
            logger.debug(
                f"Recalling position requested future time {t_req}, "
                f"returning result at last recorded time {self.time_axis[-1]}"
            )
            return agent.memory[-1]
        #print('self.time_axis',self.time_axis)
        #print('t_req',t_req)
        #print('self.time_axis[0]',self.time_axis[0])
        # check if requesting time is older than short memory
        if t_req < self.time_axis[0] :#+ self.sim.Dt: #(andrea)
            logger.warning(
                f"History memory too short to remember positions at time {t_req}, "
                f"returning result at earliest recorded time {self.time_axis[0]}"
            )
            return agent.memory[0]

        # return the interpolated position
        return self.get_position(self.time_axis, agent.memory, t_req)

    @staticmethod
    def get_position(time_axis: deque, positions: deque, t_req: float) -> np.ndarray:
        """
        Given time_axis (sorted deque/array of times) and positions (deque/list of np.array shape (3,)),
        return the interpolated position at t_req.
        """
        time_axis = np.asarray(time_axis)
        positions = np.asarray(positions)  # shape (N, 3)

        # find the right index (i such that time_axis[i] <= t_req < time_axis[i+1])
        idx = np.searchsorted(time_axis, t_req) - 1
        idx = np.clip(idx, 0, len(time_axis) - 2)

        t0, t1 = time_axis[idx], time_axis[idx + 1]
        p0, p1 = positions[idx], positions[idx + 1]
        
        # guard against division by 0 case
        if t1 == t0:
            return p0
        
        # interpolation factor
        alpha = (t_req - t0) / (t1 - t0)
        return (1 - alpha) * p0 + alpha * p1