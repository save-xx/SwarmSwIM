import numpy as np

SEED = 111

class CNNDetection:
    def __init__(self, visibility_model = None):
        self.visibility_model = visibility_model
        # genrate random seed
        self.rnd = np.random.RandomState(SEED)
        self.detections = {}

    def emulate_error (self, data, error):
        ''' Alter the input data to simulate measurment errors '''
        data += error[0]
        data += self.rnd.normal(scale=error[1])
        return data

    def _include_new_agents(self, Simulator):
        ''' check and add new agents'''
        for agent in  Simulator.agents:
            if not agent.name in self.detections:
                # add agent, empty detector and random time from last detection
                self.detections[agent.name] = [{}, self.rnd.uniform(0,agent.sensors['NNDetector']['period'])]

    def _remove_old_agents(self, Simulator):
        ''' check if an agent is no longer in simulation'''
        # fast check, compare lengths of the 2 lists to see if they match
        if len(self.detections)==len(Simulator.agents): return
        # otherwise search for leftovers
        temporary_namelist = [ag.name for ag in Simulator.agents]
        keys_to_remove= []
        for key in self.detections:             
            if not key in temporary_namelist: keys_to_remove.append(key)
        self.detections = {key: self.detections[key] for key in self.detections if key not in keys_to_remove}


    def __call__(self,Simulator):
        ''' NN sensor Callback, for each tick '''
        list_of_updates = []
        # add any newely present element to the simulator and remove old
        self._include_new_agents(Simulator)
        self._remove_old_agents(Simulator)
        # Iterate for each agent and eventually update detections
        for key_agent in self.detections:
            # get agent object
            agent = [ag for ag in Simulator.agents if ag.name == key_agent][0]
            # update timer
            self.detections[key_agent][1] += Simulator.Dt
            if self.detections[key_agent][1] < agent.sensors['NNDetector']['period'] : continue

            # else new detection is required
            self.detections[key_agent][1] = 0
            list_of_updates.append(key_agent)
            self.update_detections(Simulator, agent)
        # return a list with the names of the agents that have received an update
        return list_of_updates

    def update_detections(self,Simulator,agent):
        ''' updates relative positions of each agent '''
        for other in Simulator.agents:
            # skip self
            if other.name == agent.name: continue
            # calculate relative position, in camera setting
            rel_pos = agent.pos-other.pos
            distance = np.linalg.norm(rel_pos)
            psi_rel = np.rad2deg(np.arctan2(rel_pos[1],rel_pos[0]))%360
            alpha = (agent.psi-psi_rel)%360
            if alpha>180:alpha-=360
            beta = np.rad2deg(np.arctan2(rel_pos[2],np.linalg.norm(rel_pos[0:2])))%360
            if beta>180:beta-=360
            detection = [distance,alpha,beta]
            # apply detection 
            if self.is_detection_succesful(detection, agent):
                # If succesful 
                measured_detection = self.detection_uncertanties(detection,agent)
                self.detections[agent.name][0]= {other.name: measured_detection}
            # if not detected remove previous detection, if exist
            else: 
                if other.name in self.detections[agent.name][0]:
                    self.detections[agent.name][0].pop(other.name)

    def is_detection_succesful(self, detection, agent):
        ''' Verify is detection is invluded in the agent FoV and if it has been detected '''
        # Check if in the horizontal FoV
        if abs(detection[1])<(agent.sensors['NNDetector']['field_of_view'][0]/2): 
            return False
        # Check if in the vertical FoV
        if abs(detection[2])>(agent.sensors['NNDetector']['field_of_view'][1]/2): return False
        ## Probabilistic visibility models
        # no model, always effective
        if None==agent.sensors['NNDetector']['visibility_model']: return True
        # linear interpolation on n points 
        if agent.sensors['NNDetector']['visibility_model']=="linear":
            probability = np.interp(detection[0],
                                    agent.sensors['NNDetector']['points'][0],
                                    agent.sensors['NNDetector']['points'][1])
            if self.rnd.rand()>probability: return False 
            else: return True


    def detection_uncertanties(self, detection, agent):
        ''' Apply uncertainties of the detector to the stored output'''
        detection[0] = self.emulate_error(detection[0],agent.sensors['e_NND_distance'])
        detection[1] = self.emulate_error(detection[1],agent.sensors['e_NND_alpha'])
        detection[2] = self.emulate_error(detection[2],agent.sensors['e_NND_beta'])
        return detection