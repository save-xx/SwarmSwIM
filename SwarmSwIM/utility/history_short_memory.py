import numpy as np
from collections import defaultdict, deque
import xml.etree.ElementTree as ET
import logging
import copy


logger = logging.getLogger(__name__)
DEFAULT_LENGTH = 10


class HistoryShortMemory:
    def __init__(self, sim):
        self.sim = sim
        self.history = defaultdict(lambda: deque(maxlen=DEFAULT_LENGTH))
        self.time_axis = deque(maxlen=DEFAULT_LENGTH)
        # load length
        self.set_history_length()

    def set_history_length(self):
        """Load history length from simulation file, fallback to default if invalid."""
        try:
            tree = ET.parse(self.sim._simulation_filepath)
            root = tree.getroot()
            length_text = root.find("history_length")
            if length_text is not None and length_text.text is not None:
                self.HISTORY_LENGTH = int(length_text.text)
            else:
                raise ValueError("Missing <history_length>")
        except Exception as e:
            self.HISTORY_LENGTH = DEFAULT_LENGTH
            logger.info(
                f"Could not read/convert history_length, revert to default: {DEFAULT_LENGTH}"
            )
            logger.debug(f"Exception: {e}")

        # update existing deques to respect HISTORY_LENGTH
        self.time_axis = deque(self.time_axis, maxlen=self.HISTORY_LENGTH)
        self.history = defaultdict(lambda: deque(maxlen=self.HISTORY_LENGTH), self.history)

    def __call__(self):
        """Keep a record of all positions in the last HISTORY_LENGTH steps."""
        # update time_axis
        self.time_axis.append(self.sim.time)

        # update history
        for name, agent in self.sim.agents.items():
            # make a copy of the current position
            position = copy.deepcopy(agent.pos)
            self.history[name].append(position)

    def recall_position(self, t_req, name):
        """Recall the position of an agent at a certain time.""" 
        # check agent existence
        if name not in self.sim.agents:
            raise KeyError(f"Agent name {name} not found in the simulation")

        if not self.time_axis:  # empty history
            raise RuntimeError("History is empty, cannot recall position")

        # check if requesting time is future time compared to time_axis
        if t_req > self.time_axis[-1]:
            logger.debug(
                f"Recalling position requested future time {t_req}, "
                f"returning result at last recorded time {self.time_axis[-1]}"
            )
            return self.history[name][-1]

        # check if requesting time is older than short memory
        if t_req < self.time_axis[0]:
            logger.warning(
                f"History memory too short to remember positions at time {t_req}, "
                f"returning result at earliest recorded time {self.time_axis[0]}"
            )
            return self.history[name][0]

        # return the interpolated position
        return self.get_position(self.time_axis, self.history[name], t_req)

    @staticmethod
    def get_position(time_axis, positions, t_req):
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

        # interpolation factor
        alpha = (t_req - t0) / (t1 - t0)
        return (1 - alpha) * p0 + alpha * p1