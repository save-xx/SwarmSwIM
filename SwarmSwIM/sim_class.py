import numpy as np
import copy
import logging
from .agent_class import Agent
from . import sim_functions
from .utility.history_short_memory import HistoryShortMemory


logger = logging.getLogger(__name__)


class Simulator():
    def __init__(self, timeSubdivision=1.0, sim_xml="simulation.xml"):
        """
        Simulation Object
        - timeSubdivision: (float), unit in seconds, time interval used for each simulation step.
        - sim_xml: (string) name of XML file describing the simulation parameters
        """
        self.step_count = 0
        self.time = 0
        self.Dt = timeSubdivision

        self.memory = None
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
        self._agents_from_file()

    def __iter__(self):
        """Overload the iterator to provide agents"""
        return iter(self.agents.items())

    def __getitem__(self, key):
        """Allow dict-style access to agents"""
        return self.agents[key]
    
    def __setitem__(self, key, value):
        """Allow dict-style assignment"""
        if not isinstance(key, str):
            raise TypeError(f"Agent name must be a string type. Passed {key}")
        if not isinstance(value, Agent):
            raise TypeError(f"Agent object must be a Agent instance. Passed a {type(value)}")
        self.agents[key] = value

    def __contains__(self, key):
        """Enable `if key in S:` syntax"""
        return key in self.agents

    def __len__(self):
        """Number of agents"""
        return len(self.agents)

    def _agents_from_file(self):
        """Load agents based on simulation XML specification."""
        data = sim_functions.parse_agents(self._simulation_filepath)
        for key, value in data.items():
            self._add(Agent(key,value[0],value[1],value[2],self.seed))

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
        self.step_count += 1
        self.time += self.Dt
        # execute pre step plugins
        responses_pre = self.execute_plugins(self.plugins_calls_prestep)
        # execute physiscs
        for agent in self.agents.values():
            agent.tick()
        # update the short term memory of positions
        if self.has_memory:
            self.memory()
        # execute post step plugins
        responses_post = self.execute_plugins(self.plugins_calls_poststep)
        # save data
        if hasattr(self, "save_plugin"):
            self.save_plugin()
        # return all plugins outputs (combine dictionaries)
        return responses_pre | responses_post # <- fuse 2 dict into one


    def rel_pos(self, A : Agent, B :Agent):
        ''' Measure relative distance of 2 agents (A and B), as vector A to B'''
        return (B.pos-A.pos)
    
    
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

