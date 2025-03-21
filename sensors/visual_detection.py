import numpy as np

SEED = 111

class CNNDetection:
    def __init__(self, rnd = None, visibility_model = None):
        self.visibility_model = visibility_model
        # genrate random seed
        if rnd: self.rnd = rnd
        else: self.rnd = np.random.default_rng()

    def emulate_error (self, data, error):
        ''' Alter the input data to simulate measurment errors '''
        data += error[0]
        data += self.rnd.normal(scale=error[1])
        return data

    def __call__(self,Simulator):
        ''' NN sensor Callback, for each tick '''
        list_of_updates = []
        # add any newely present element to the simulator and remove old
        # Iterate for each agent and eventually update detections
        for agent in Simulator.agents:
            # update timer
            agent.NNDetector['time_lapsed'] += Simulator.Dt
            if agent.NNDetector['time_lapsed'] < agent.sensors['NNDetector']['period'] : continue
            # else new detection is required
            agent.NNDetector['time_lapsed'] = 0
            list_of_updates.append(agent.name)
            self.update_detections(Simulator, agent)
        # return a list with the names of the agents that have received an update
        return list_of_updates

    def update_detections(self,Simulator,agent):
        ''' updates relative positions of each agent '''
        for other in Simulator.agents:
            # skip self
            if other.name == agent.name: continue
            # calculate relative position, in camera setting
            rel_pos = other.pos-agent.pos
            distance = np.linalg.norm(rel_pos)
            psi_rel = np.rad2deg(np.arctan2(rel_pos[1],rel_pos[0]))%360
            alpha = (psi_rel-agent.psi)%360
            if alpha>180:alpha-=360
            beta = -np.rad2deg(np.arctan2(rel_pos[2],np.linalg.norm(rel_pos[0:2])))%360
            if beta>180:beta-=360
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