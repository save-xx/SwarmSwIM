import numpy as np
import copy
import logging
from SwarmSwIM import Agent
from . import sim_functions
from SwarmSwIM import HistoryShortMemory

logger = logging.getLogger(__name__)
# Short term history memory of all agents, to consider acoustic effects
HISTORY_MEMORY = 2 # seconds
C_SOUND = 1500 # m/s

class Simulator():
    def __init__(self, timeSubdivision=1.0, sim_xml="simulation.xml"):
        """
        Simulation Object
        - timeSubdivision: (float), unit in seconds, time interval used for each simulation step.
        - sim_xml: (string) name of XML file describing the simulation parameters
        """
        self._Dt = 0
        self._hist_length = 2 # minimum length

        self.time = 0
        self.Dt = timeSubdivision

        self.history = None
        self.agents = {}

        self.plugins_calls_prestep = {}
        self.plugins_calls_poststep = {}

        # extrapolate absolute path of simultation description
        # priority: absolute > user script dir > execution dir > package dir
        self._simulation_filepath = sim_functions.get_xml_path(sim_xml)
        # extract seed
        self.seed = sim_functions.get_seed(self._simulation_filepath)
        # initialize randomness < if self.seed is None then a random rng is made
        self.rnd = np.random.default_rng(self.seed)
        # initialize file agents
        self.agents_from_file()
        
    def agents_from_file(self):
        """Load agents based on simulation XML specification."""
        data = sim_functions.parse_agents(self._simulation_filepath)
        for key, value in data.items():
            self._add(Agent(key,value[0],value[1],value[2],self.seed))

    def __iter__(self):
        """Overload the iterator to provide agents"""
        return iter(self.agents.items())

    @property
    def Dt(self):
        return self._Dt
    
    @Dt.setter
    def Dt(self, input):
        self._Dt = input
        self._hist_length = max(2,int(np.ceil(HISTORY_MEMORY/input))) # minimum 2 cells

    # --------------------------
    # Short term memory handling
    # --------------------------

    @property
    def has_memory(self):
        return self.memory is not None
    
    def enable_memory(self):
        if not self.has_memory:
            self.memory = HistoryShortMemory(self)

    def disable_memory(self):
        self.memory = None

    # ------------------
    # Add/ Remove agents
    # ------------------

    # Internal methods to add a single agent from the simulation
    def _add(self, new_agent):   
        """Add an Agent to the simulation.""" 
        if not isinstance(new_agent, Agent):
            logger.warning(f"Can't add instance: object of type {type(new_agent).__name__} is not an Agent instance.")
            return
        # avoid clones
        if new_agent.name in self.agents:
            logger.warning(f"Name {new_agent.name}, already used, agent not added")
            return
        # enforce Dt to agent
        new_agent.Dt = self.Dt
        # initialize history (assume no movment in the past)
        self.history[new_agent.name] = [copy.deepcopy(new_agent.pos)]*self._hist_length
        # Add to simulation dictionary
        self.agents[new_agent.name] = new_agent

    # Internal methods to remove a single agent from the simulation
    def _remove(self,new_agent):
        if not isinstance(new_agent, Agent):
            logger.warning(f"Can't add instance: object of type {type(new_agent).__name__} is not an Agent.")
            return
        # skip agent if not in list of agents
        if not new_agent.name in self.agents: 
            logger.warning(f"Agent name {new_agent.name}, is not present in simulation already.")
            return
        # remove instance from all dictionaries
        self.agents.pop(new_agent.name, None)
        self.history.pop(new_agent.name, None)

    # Methods to add or remove agent(s) from the simulation
    def remove(self,*args):
        ''' iterate  on add args to remove each agent individually'''
        # direct to single entries    
        if not args: return 
        for agent in args: 
            self._remove(agent)

    def add(self,*args):
        ''' iterate  on add args to add each agent individually'''
        # direct to single entries    
        if not args: return  
        for agent in args: 
            self._add(agent)

    #------------------
    
    @staticmethod
    def execute_plugins(dict_of_plugins):
        # Call all list plugins
        responses = {}
        for key, plugin in dict_of_plugins.items():
            # raise warining if callable missing
            if not callable(plugin):
                logger.warning(f"Plugin {plugin.__class__.__name__} is not callable. Skipping.")
                continue
            try:
                resp = plugin()  # Calls the plugin's __call__ method
                responses[key] = resp
            except Exception as e:
                logger.error(f"Error during plugin call {plugin.__class__.__name__}: {e}")
        return responses
    
    ## Execution function ##
    def tick(self):
        """Advance one step of simulation."""
        # update time
        self.time += self.Dt
        # execute pre step plugins
        responses_pre = self.execute_plugins(self.plugins_calls_prestep)
        # execute physiscs
        for agent in self.agents.values():
            agent.tick()
        # update the short term memory of positions
        if self.has_memory:
            self.memory()
        self.update_history()
        # execute post step plugins
        responses_post = self.execute_plugins(self.plugins_calls_poststep)
        # return all plugins outputs (combine dictionaries)
        return responses_pre | responses_post # <- fuse 2 dict into one

    # Subfunction of the main tick
    def update_history(self):
        """Keep a record of all positions in the last HISTORY_MEMORY seconds."""
        for agent in self.agents.values():
            position = copy.deepcopy(agent.pos)
            hist = self.history[agent.name]
            hist.append(position)
            # Ensure fixed history length
            if len(hist)>self._hist_length:
                self.history[agent.name].pop(0)
            # Edge case: history had only one entry, fill it
            if 1 == len(hist):
                self.history[agent.name]= [position] * self._hist_length

    def rel_pos(self, A : Agent, B :Agent):
        ''' Measure relative distance of 2 agents (A and B), as vector A to B'''
        return (B.pos-A.pos)
    
    def acoustic_range(self, A : Agent, B :Agent):
        ''' 
        Return the acoustic range between 2 agents (A and B), measured in A.  
        Accounts for Time of Flight between A and B.
        '''
        d0 = np.linalg.norm(self.rel_pos(A,B))
        delay_seconds= d0/C_SOUND
        if delay_seconds>=HISTORY_MEMORY: raise MemoryError ("The ditance delays exeed the history memory, \
                                                                increase the memory interval HISTORY_MEMORY")
        times = np.arange(-(len(self.history[B.name]) - 1) * self.Dt, self.Dt, self.Dt)
        dists = [np.linalg.norm(Bhist - A.pos) for Bhist in self.history[B.name]] 
        perfect_distance = np.interp(-delay_seconds,times,dists)                        # Distance assumed no error
        measured_distance = A.emulate_error( perfect_distance, A.sensors['e_ac_range'] )   # Added measurment
        return measured_distance

    def OWTT_acoustic_range(self, A : Agent, B :Agent):
        ''' Returns the One Way Time Traver Ranging, accounting for clock drift error'''
        ideal_range = self.acoustic_range(A,B)
        drift_variance = (A.internal_clock - B.internal_clock) * C_SOUND
        return ideal_range + drift_variance

    def doppler(self,A,B):
        ''' 
        Returns  the velocity component projected on the AB axis, as if estimated via acoustic Doppler shift.
        - A and B: Agent instances  
        - Measurament obtaines as if captured in A  
        - Accounts for message time duration, averaging the measurament on the time interval.
        '''
        msg_dt = A.sensors['ac_msg_length']
        ## Approxiate length in messages (minimum 2 considered)
        elements = int(np.ceil(msg_dt/self.Dt)+1)
        ## element shift due to acoustic delay
        delay_seconds= np.linalg.norm(self.rel_pos(A,B))/C_SOUND
        delay_steps = int(delay_seconds//self.Dt)
        if elements+delay_steps > self._hist_length: 
            raise MemoryError ("The message length exeeds the history memory, \
                                increase the memory interval HISTORY_MEMORY")
        ranges = []
        for i in range(elements):
            # collect distances over msg time interval (minimum 2)
            distance = (self.history[B.name][-1-i-delay_steps]-self.history[A.name][-1-i])
            ranges.append(np.linalg.norm(distance))
        perfect_doppler = np.mean(-np.diff(ranges)/self.Dt)          # ideal measurment of doppler
        measured_doppler = A.emulate_error( perfect_doppler, A.sensors['e_ac_doppler'] ) 
        return measured_doppler
    
    @property
    def states(self):
        '''return state of each agent in a dictionary'''
        output = {}
        for agent in self.agents:
            output[agent.name]=[agent.pos[0],agent.pos[1],agent.pos[2],agent.psi]
        return output

if __name__=="__main__":

    S = Simulator(0.1)
    A1 = S.agents[0]
    A1.cmd_fhd(0.0,0.,0.)

    for i in range(80):
        S.tick()
        # print(f'{A1.pos[0]:.6f},{A1.pos[1]:.6f}')

    print('-----')
    print (S.OWTT_acoustic_range(S.agents[0],S.agents[1]))
    print (S.acoustic_range(S.agents[0],S.agents[1]))
    print (S.doppler(S.agents[0],S.agents[1]))
