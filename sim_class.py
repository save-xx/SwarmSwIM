import numpy as np
import copy

from agent_class import Agent
import sim_functions

# Short term history memory of all agents, to consider acoustic effects
HISTORY_MEMORY = 2 # seconds
C_SOUND = 1500 # m/s
SEED = 111 # random initialization

class Simulator():
    def __init__(self,Dt,sim_xml="simulation.xml"):
        self._Dt = 0
        self._hist_length = 2 # minimum length
        self.rnd = np.random.RandomState(SEED)

        self.time = 0
        self.Dt = Dt
        self.history = {}
        self.agents = []
        # if None is passed, use default
        if sim_xml==None: sim_xml="simulation.xml"
        # initialize file agents
        self.agents_from_file(sim_xml)

        # load enviroment parameters:
        self.environment = sim_functions.parse_envrioment_parameters(sim_xml)
        # initialize current classes:
        if self.environment['is_vortex_currents']:
            self.vortex_field =  sim_functions.VortexField(
                self.environment['vortex_currents_density'],
                self.environment['vortex_currents_intensity'],
                self.rnd)
        if self.environment['is_noise_currents']:
            self.turbolent_noise = sim_functions.TimeNoise(
                self.time, 
                self.environment['noise_currents_freq'],
                self.environment['noise_currents_intensity'],
                self.rnd
            )
        
    def agents_from_file(self,sim_xml):
        data = sim_functions.parse_agents(sim_xml)
        for key, value in data.items():
            self._add(Agent(key,0.1,value[0],value[1],value[2]))

    @property
    def Dt(self):
        return self._Dt

    @Dt.setter
    def Dt(self,input):
        self._Dt = input
        self._hist_length = max(2,int(np.ceil(HISTORY_MEMORY/input))) # minimum 2 cells

    def _add(self, new_agent):   
        ''' Private function Add an Agent to the simulation ''' 
        # add new agent to list
        if not type(new_agent) == Agent:
            print("ERROR: only Agent type object can be added, operation aborted")
            return
        # avoid clones
        for agent in self.agents:
            if agent.name==new_agent.name: 
                print(f"ERROR: Agent already exisit with name {agent.name}, operation aborted")
                return
        # Add to simulation
        self.agents.append(new_agent)
        # Update internal time and Dt
        self.agents[-1].Dt = self.Dt
        # initialize history (assume no movment in the past)
        self.history[new_agent.name] = [copy.deepcopy(self.agents[-1].pos)]*self._hist_length

    def _remove(self,new_agent):
        if not type(new_agent) == Agent:
            print("ERROR: only Agent type object can be removed, operation aborted")
            return
        # skip agent if not in list of agents
        if not new_agent in self.agents: return
        # remove instance from all list
        self.agents.remove(new_agent)
        del self.history[new_agent.name]

    def remove(self,*args):
        ''' iterate  on add args to remove each agent individually'''
        # direct to single entries    
        if args: 
            for agent in args: 
                self._remove(agent)

    def add(self,*args):
        ''' iterate  on add args to add each agent individually'''
        # direct to single entries    
        if args: 
            for agent in args: 
                self._add(agent)
        
    def tick(self):
        ''' execute a step of simulation for all the loaded agents'''
        # update time
        self.time += self.Dt
        # execute physiscs
        for agent in self.agents:
            agent.tick()
        # add current disturbances
        self.calculate_currents()
        # update the short term memory of positions
        self.update_history()

    def update_history(self):
        ''' keep a record of all positions in the last HISTORY_MEMORY seconds '''
        for agent in self.agents:
            position = copy.deepcopy(agent.pos)
            self.history[agent.name].append(position)
            if len(self.history[agent.name])>self._hist_length:
                self.history[agent.name].pop(0)
            if 1==len(self.history[agent.name]):
                self.history[agent.name]= [position] * self._hist_length

    def calculate_currents(self):
        ''' apply position disturbance, based on current, to each agent'''
        global_current = np.array([0.,0.])
        # add global constant current
        if self.environment['is_uniform_current']: global_current += self.environment['uniform_current']
        # add global time dependant wave current
        if self.environment['is_global_waves']:
            for waves in self.environment['global_waves']:
                global_current += sim_functions.global_waves(self.time , 
                                           waves['amplitude'], waves['frequency'],
                                           waves['direction'], waves['shift'])
        global_current = np.append(global_current,0)
        # Add space dependant currents
        for agent in self.agents:
            local_current = np.array([0.,0.])
            # add turbolent field disturbance
            if self.environment['is_vortex_currents']:
                local_current += self.vortex_field.current_vortex_calculate(agent)
            # add localized turbolent noise
            if self.environment['is_noise_currents']:
                local_current += self.turbolent_noise.calculate_noises(self.time,agent)
            # add localized waves
            if self.environment['is_local_waves']:
                for waves in self.environment['local_waves']:
                    local_current += sim_functions.local_waves(self.time , agent, 
                                            waves['amplitude'], waves['wavespeed'],
                                            waves['wavelength'], waves['direction'],
                                            waves['shift'])
            # add space independent disturbances
            agent.pos += global_current*self.Dt
            # add space dependant disturbances
            agent.pos += np.append(local_current,0)*self.Dt

    def rel_pos(self,A,B):
        ''' measure relative distance of 2 agents, as vector A to B'''
        return (B.pos-A.pos)
    
    def acoustic_range(self,A,B):
        ''' return  the acoustic range, measured in A. A in the current time instance
        and B, in a past instance, relative to when the acoustic message was initated'''
        d0 = np.linalg.norm(self.rel_pos(A,B))
        delay_seconds= d0/C_SOUND
        if delay_seconds>=HISTORY_MEMORY: raise MemoryError ("The ditance delays exeed the history memory, \
                                                                increase the memory interval HISTORY_MEMORY")
        times = np.arange(-(len(self.history[B.name]) - 1) * self.Dt, self.Dt, self.Dt)
        dists = [np.linalg.norm(Bhist - A.pos) for Bhist in self.history[B.name]] 
        perfect_distance = np.interp(-delay_seconds,times,dists)                        # Distance assumed no error
        measured_distance = A.emulate_error( perfect_distance, A.sensors['e_ac_range'] )   # Added measurment
        return measured_distance

    def OWTT_acoustic_range(self,A,B):
        ''' return the One Way Time Traver Ranging accounting for clock drift'''
        ideal_range = self.acoustic_range(A,B)
        drift_variance = (A.internal_clock - B.internal_clock) * C_SOUND
        return ideal_range + drift_variance

    def doppler(self,A,B):
        ''' return doppler shift as velocity. 
        msg_dt is the transmission time of the message
        doppler is considered measured in A - only for long range
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
        perfect_doppler = np.mean(np.diff(ranges)/self.Dt)          # ideal measurment of doppler
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
