import numpy as np
import xml.etree.ElementTree as ET
import logging
from dataclasses import dataclass, field
from SwarmSwIM.sim_functions import parse_matrix


logger = logging.getLogger(__name__)
EXPECTED_TAGS = ["period", "field_of_view", "visibility_model"]


def activate_Detector(simulation, detector_name: str = "detector"):
    """Activate detection plugin to simulation."""
    rnd = np.random.default_rng(simulation.seed) # new rnd based on same seed
    detector_inst = Detection(simulation, detector_name, rnd)
    simulation.plugins_calls_poststep[detector_name] = detector_inst


@dataclass
class Generic_detector_memory:
    # static settings
    period: float = 0.0
    field_of_view: np.ndarray = field(default_factory=lambda: np.zeros([2,2]))
    visibility_model: str = "none"
    points: np.ndarray = field(default_factory=lambda: np.empty((0,)))
    e_distance: np.ndarray = field(default_factory=lambda: np.zeros(2))
    e_alpha: np.ndarray = field(default_factory=lambda: np.zeros(2))
    e_beta: np.ndarray = field(default_factory=lambda: np.zeros(2))
    # runtime parameters
    since_last_detection: float = 0.0
    latest_detections: dict = field(default_factory=dict)


def check_agent_root(path, detector="detector"):
    """Verify that the agent file contains the specified detector tag."""
    tree = ET.parse(path)
    root = tree.getroot()
    sensor_root = root.find("sensors")
    # if no sensor is specified skip
    if sensor_root is None: 
        logger.warning("The XML does not contain a <sensors> element.")
        return
    detector_root = sensor_root.find(detector)
    # the specific detector is not mounted, skip
    if detector_root is None:
        logger.warning(f"The XML does not contain a <{detector}> element.")
        return
    actual_tags = [child.tag for child in detector_root]
    missing_tags = [tag for tag in EXPECTED_TAGS if tag not in actual_tags]
    # if mandatory tags are missing, then raise an error
    if missing_tags:
        raise ValueError(f"Missing tags in the <{detector}> detector: {missing_tags}")
    return detector_root


class Detection:
    def __init__(self, simulation, detector_name="detector", rnd=None):
        # genrate random seed
        if rnd is not None: 
            self.rnd = rnd
        else: 
            self.rnd = np.random.default_rng()
        
        self.detector_name = detector_name
        self.sim = simulation
        # initialize each agent
        for _, agent in self.sim:
            self.initiate_agent(agent)

    def initiate_agent(self, agent):
        detector_root = check_agent_root(agent._agent_filepath, self.detector_name)
        # create attribute if needed (do regardless of the sensor presence)
        if not hasattr(agent, "detectors"):
            setattr(agent, "detectors", {})
        # skip agents without the sensor
        if detector_root is None:
            return
        # add detector to agent
        agent.detectors[self.detector_name] = Generic_detector_memory()
        # parse values
        self.parse_detector(agent, detector_root)

    def parse_detector(self, agent, detector_root):
        """Parse a detector sensor description from an agent and populate the class."""
        detector = agent.detectors[self.detector_name]
        detector.period = float(detector_root.find('period').text)
        detector.field_of_view = parse_matrix(detector_root.find('field_of_view'))
        detector.visibility_model = detector_root.find('visibility_model').text
        # optional points
        if detector_root.find('points') is not None:
            detector.points = parse_matrix(detector_root.find('points'))
        # parse noise errors
        if detector_root.find('e_distance'):
            detector.e_distance = parse_matrix(detector_root.find('e_distance'))
        if detector_root.find('e_alpha'):
            detector.e_alpha = parse_matrix(detector_root.find('e_alpha'))
        if detector_root.find('e_beta'):
            detector.e_beta = parse_matrix(detector_root.find('e_beta'))
        # save changes
        agent.detectors[self.detector_name] = detector

    def emulate_error (self, data, error):
        ''' Alter the input data to simulate measurment errors '''
        data += error[0]
        data += self.rnd.normal(scale=error[1])
        return data

    def __call__(self):
        """Execute detector logic for eact step."""
        dict_of_updates = {}
        # add any newely present element to the simulator and remove old
        # Iterate for each agent and eventually update detections
        for name, agent in self.sim:
            # skip if dectector is missing for the agent
            if not self.detector_name in agent.detectors:
                continue
            # update timer
            detector = agent.detectors[self.detector_name]
            agent.detectors[self.detector_name].since_last_detection += self.sim.Dt
            if detector.since_last_detection < detector.period:
                continue
            # else new detection is required
            agent.detectors[self.detector_name].since_last_detection = 0.0
            self.update_detections(agent, detector)
            # update return with the new detections
            dict_of_updates[name] = agent.detectors[self.detector_name].latest_detections
        # return a list with the names of the agents that have received an update
        return dict_of_updates

    def update_detections(self, agent, detector):
        ''' updates relative positions of each agent '''
        # remove all previous detections
        agent.detectors[self.detector_name].latest_detections.clear()
        for name, other in self.sim:
            # skip self
            if name == agent.name:
                continue
            # calculate relative position, in camera setting
            rel_pos = other.pos - agent.pos
            distance = np.linalg.norm(rel_pos)
            # avoid numerical issues: can't detect overlapping agents
            if distance == 0:
                continue
            # try distance failure probability
            if not self.distance_model(detector, distance):
                continue
            # calculate alpha and beta angles of detection
            psi_rel = np.rad2deg(np.arctan2(rel_pos[1],rel_pos[0]))%360
            # horizontal angle of detection
            alpha = (psi_rel - agent.psi)%360
            alpha -= 360 if alpha > 180 else 0
            # vertical angle of detection
            ratio = np.clip(rel_pos[2] / distance, -1.0, 1.0)
            beta = -np.rad2deg(np.arcsin(ratio))%360
            beta -= 360 if beta > 180 else 0
            # verify FoV
            if not self.check_field_of_view(detector, alpha, beta):
                continue 
            # from this point the detection is considered succesfull
            # construct result with sensor noise
            result = self.apply_sensor_noise(detector, distance, alpha, beta)
            agent.detectors[self.detector_name].latest_detections[name] = result


    def distance_model(self, detector, distance):
        """Checks if the detection is succesfull, based on distance."""
        model = detector.visibility_model.lower() if detector.visibility_model else ''
        # No check, alwasy detect
        if model  in ['','none']:
            return True
        # spliwise linear model adopted
        if model == 'linear':
            pts = detector.points
            # Ensure detector.points has correct shape
            if pts.shape[0] != 2 or pts.size == 0:
                raise ValueError("detector.points must be a (2, N) not empty array for 'linear' model.")
            # linear interpolation over distance to get detection probability
            prob = float(np.interp(distance, pts[0], pts[1]))
            prob = float(np.clip(prob, 0.0, 1.0))
            return self.rnd.random() <= prob
        # if none of the options
        logger.debug(f"Unknown visibility_model '{detector.visibility_model}'")
        return False

    def check_field_of_view(self, detector, alpha, beta):
        """Verify if detection is in the FoV of the agent."""
        fov = detector.field_of_view
        if fov.shape != (2, 2):
            raise ValueError(f"field_of_view must be 2x2, got {fov.shape}")
        # Return True if in the FoV, False otherwise
        return (fov[0, 0] <= alpha <= fov[0, 1]) and (fov[1, 0] <= beta <= fov[1, 1])


    def apply_sensor_noise(self, detector, dist, alpha, beta):
        """Add error to the measuraments"""
        dist_n = self.emulate_error(dist, detector.e_distance)
        alpha_n = self.emulate_error(alpha, detector.e_alpha)
        beta_n = self.emulate_error(beta, detector.e_beta)
        return {'distance': dist_n, 'alpha': alpha_n, 'beta': beta_n}

