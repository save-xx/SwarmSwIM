import numpy as np
import xml.etree.ElementTree as ET
import logging
from dataclasses import dataclass, field

from ..sim_functions import parse_matrix

logger = logging.getLogger(__name__)
EXPECTED_TAGS = ["period", "field_of_view", "visibility_model"]

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
        logger.debug("The XML does not contain a <sensors> element.")
        return
    detector_root = sensor_root.find(detector)
    # the specific detector is not mounted, skip
    if detector_root is None:
        logger.debug(f"The XML does not contain a <{detector}> element.")
        return
    actual_tags = [child.tag for child in detector_root]
    missing_tags = [tag for tag in EXPECTED_TAGS if tag not in actual_tags]
    # if mandatory tags are missing, then raise an error
    if missing_tags:
        raise ValueError(f"Missing tags in the <{detector}> detector")
    return detector_root

class Detection:
    def __init__(self, simulation, detector_name="detector", rnd=None):
        # genrate random seed
        if rnd: 
            self.rnd = rnd
        else: 
            self.rnd = np.random.default_rng()
        
        self.detector_name = detector_name
        self.sim = simulation
        # initialize each agent
        for name, agent in self.sim():
            self.initiate_agent(agent)

    def initiate_agent(self, agent):
        detector_root = check_agent_root(agent._agent_filepath)
        # skip agents without the sensor
        if detector_root is None:
            return
        # add detector to agent
        setattr(agent, self.detector_name, Generic_detector_memory())
        # parse values
        self.parse_detector(agent, detector_root)

    def parse_detector(self, agent, detector_root):
        """Parse a detector sensor description from an agent and populate the class."""
        detector = getattr(agent, self.detector_name)
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
            detector = getattr(agent, self.detector_name)
            # update timer
            detector.since_last_detection += self.sim.Dt
            if detector.since_last_detection < detector.period:
                continue
            # else new detection is required
            detector.since_last_detection = 0.0
            dict_of_updates.append(agent.name)
            self.update_detections(agent, detector)
        # return a list with the names of the agents that have received an update
        return dict_of_updates

    def update_detections(self,agent,detector):
        ''' updates relative positions of each agent '''
        for name, other in self.sim:
            # skip self
            if name == agent.name:
                continue
            # skip if dectector is missing for the agent
            if hasattr(agent, self.detector_name):
                continue
            # calculate relative position, in camera setting
            rel_pos = other.pos - agent.pos
            distance = np.linalg.norm(rel_pos)
            # avid numerical issues: can't detect overlapping agents
            if distance == 0:
                continue
            # try distance failure probability
            if not self.distance_model(detector, distance):
                continue
            # verify FoV compatibility
            psi_rel = np.rad2deg(np.arctan2(rel_pos[1],rel_pos[0]))%360
            # horizontal angle of detection
            alpha = (psi_rel - agent.psi)%360
            alpha -= 360 if alpha > 180 else 0
            # vertical angle of detection
            beta = -np.rad2deg(np.arcsin(rel_pos[2] / distance))%360
            beta -= 360 if beta > 180 else 0
            # TODO continue from here

            detection = [distance,alpha,beta]
            # apply detection 
            if self.is_detection_succesful(detection, agent):
                # If succesful 
                measured_detection = self.detection_uncertanties(detection,agent)
                agent.NNDetector[other.name]=measured_detection
            # if not detected remove previous detection, if exist
            else: 
                if other.name in agent.NNDetector:
                    agent.NNDetector.pop(other.name)
        # remove deleted agents
        agent_names = {agent.name for agent in Simulator.agents}
        agent.NNDetector = { k: v for k, v in agent.NNDetector.items() if k in agent_names or k=="time_lapsed"} 

    def distance_model(self, detector, distance):
        """Checks if the detection is succesfull, based on distance."""
        model = detector.visibility_model.lower() if detector.visibility_model else ''
        # No check, alwasy detect
        if model  in ['','none']:
            return True
        # spliwise linear model adopted
        if model == 'linear':
            # Ensure detector.points has correct shape
            if detector.points.shape[0] != 2:
                raise ValueError("detector.points must be a (2, N) array for 'linear' model.")
            # linear interpolation over distance to get detection probability
            probability = np.interp(distance, detector.points[0], detector.points[1])
            return self.rnd.random() <= probability

    def check_field_of_view(self):
        """Verify if detection is in the FoV of the agent."""
        pass

    def is_detection_succesful(self, detection, agent):
        ''' Verify is detection is invluded in the agent FoV and if it has been detected '''
        # Check if in the horizontal FoV
        if abs(detection[1])>(agent.sensors['NNDetector']['field_of_view'][0]/2): return False
        # Check if in the vertical FoV
        if abs(detection[2])>(agent.sensors['NNDetector']['field_of_view'][1]/2): return False
        ## Probabilistic visibility models
        # no model, always effective
        if None == agent.sensors['NNDetector']['visibility_model'] or "none" == agent.sensors['NNDetector']['visibility_model']: return True
        # linear interpolation on n points 
        if agent.sensors['NNDetector']['visibility_model']=="linear":
            probability = np.interp(detection[0],
                                    agent.sensors['NNDetector']['points'][0],
                                    agent.sensors['NNDetector']['points'][1])
            if self.rnd.random()>probability: return False 
            else: return True


    def detection_uncertanties(self, detection, agent):
        ''' Apply uncertainties of the detector to the stored output'''
        detection[0] = self.emulate_error(detection[0],agent.sensors['e_NND_distance'])
        detection[1] = self.emulate_error(detection[1],agent.sensors['e_NND_alpha'])
        detection[2] = self.emulate_error(detection[2],agent.sensors['e_NND_beta'])
        return detection