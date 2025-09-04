import numpy as np
import copy
from collections import defaultdict
from collections.abc import Iterable
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, asdict, is_dataclass
from typing import Any
import itertools

from SwarmSwIM.sim_functions import parse_matrix

# generate unique ids for each msg
global_msg_id = itertools.count()
# Default values (if not specified)
SPEED_OF_SOUND = 1500. # m/s
MAX_RANGE = 2000. # m


def _to_float(text, default=0.0):
    """Turn single value tag to float"""
    if text is None or text.strip() == "":
        return default
    return float(text)



@dataclass
class AcousticMsgs:
    id: set[int] = field(default_factory=set)
    sender: str | None =  None
    payload: Any = None
    intact: bool = True
    ToD_raw: float = -1.0
    ToD_exact: float = -1.0
    ToA_raw: float = -1.0
    ToA_exact: float = -1.0
    pos_at_detection: np.ndarray | None = None
    perfect_range: float = -1.0
    ping_range: float = -1.0
    doppler_velocity: float = 0.0

@dataclass
class AgentChannel:
    status: bool = True
    release_time: float = -1.0
    incoming: AcousticMsgs = field(default_factory=AcousticMsgs)
    drift: float = 0.0
    e_range: np.ndarray = field(default_factory=lambda: np.zeros(2))
    e_doppler: np.ndarray = field(default_factory=lambda: np.zeros(2))
    e_delay: np.ndarray = field(default_factory=lambda: np.zeros(2))
    sync_gap: float = 0.0


def activate_Acoustic(simulation, acoustic_name="acoustic"):
    """Activate acoustic channel plugin to simulation."""
    acoustic_inst = AcousticChannel(simulation, acoustic_name)
    # Initiate required short memory if not already active
    if not simulation.has_memory:
        simulation.enable_memory()
    # Add to post-step operations
    simulation.plugins_calls_poststep[acoustic_name] = acoustic_inst
    # return handler to sent messages on this channel
    return acoustic_inst


class AcousticChannel:
    def __init__(
        self, 
        simulation, 
        channel_name = "acoustic",
        agent_selection: list | None = None,
        c_sound = SPEED_OF_SOUND, 
        max_range = MAX_RANGE
        ):
        # private parameters to bag
        self._event_to_save = {}
        self._sent_to_save = []
        
        #
        self.sim = simulation
        self.C_SOUND = c_sound
        self.MAX_RANGE = max_range
        self.channel_name = channel_name

        # collection of active messages
        self.active_msgs = {}

        # create random seed
        self.rnd = simulation.rnd.spawn(1)[0]
        # find and unpack parameters from simulation xml
        acoustic_root = self.check_root(simulation._simulation_filepath, channel_name)
        self.unpack_acoustic_channel(acoustic_root)
        # add attribute to all agents (even if unused)
        for _, agent in simulation:
            setattr(agent, "acoustic_channels", defaultdict(dict))
        # create active agent sub-dictionary
        self.generate_agents_dict(simulation, agent_selection)
        # initiate each agent
        for _, agent in self.sim.agents.items():
            self.populate_acoustic_channel(channel_name, agent)
        
    def emulate_error (self, data: float, error: np.ndarray) -> float:
        ''' Alter the input data to simulate measurment errors '''
        data += error[0]
        data += self.rnd.normal(scale=error[1])
        return data


    def generate_agents_dict(self, simulation, agent_selection):
        """generate dict as subgroup of agents involved in the channel"""
        self.agents_dict = {}
        if agent_selection:
            # check that agent_selection is Iterable 
            if not isinstance(agent_selection, Iterable): 
                raise ValueError(f"agent_selection must be Iterable: {agent_selection}")
            # check that the input is a list of strings
            if not all(isinstance(name, str) for name in agent_selection):
                raise ValueError(f"all elements of agent_selection must be name strings")
            
            for name in agent_selection:
                # skip if there is no name correspondence
                if not name in simulation.agents:
                    continue
                # create a reduced dictionary of agents
                self.agents_dict[name] = simulation.agents[name]

        else:
            # add all agents
            self.agents_dict = simulation.agents


    @staticmethod
    def check_root(path, channel_name):
        """Verify that the simulation file contains the envrioment tag."""
        tree = ET.parse(path)
        root = tree.getroot()
        root = root.find(channel_name)
        if root is None: 
            raise ValueError(f"The XML does not contain a <{channel_name}> element.")
        return root


    def unpack_acoustic_channel(self, root):
        """read parameters for the specified channel."""
        # update speed of sound if specified
        c_update = _to_float(root.find("speed_of_sound").text, default=self.C_SOUND)
        self.C_SOUND = c_update if c_update > 0 else self.C_SOUND
        # update max range if specified
        max_update = _to_float(root.find("max_range").text, default=self.MAX_RANGE)
        self.MAX_RANGE = max_update if max_update  > 0 else self.MAX_RANGE
        # load noise values range
        self.e_range = parse_matrix(root.find('e_acoustic_range'))
        if self.e_range.size == 0:
            self.e_range = np.zeros(2)
        # load noise values doppler
        self.e_doppler = parse_matrix(root.find('e_doppler'))
        if self.e_doppler.size == 0:
            self.e_doppler = np.zeros(2)
        # load noise values computational delay
        self.computational_delay = _to_float(root.find("delay_acoustic_send").text)

        self._e_delay = parse_matrix(root.find('e_delay'))
        if self._e_delay.size == 0:
            self._e_delay = np.zeros(2)
        # load drift value
        self._PPM  = _to_float(root.find("drift").text)
        # load syncronization gap between agent
        self._sync_gap = _to_float(root.find("sync_gap").text)

    
    def populate_acoustic_channel(self, channel_name: str, agent):
        """populate each agent channel"""
        # initialize channel status as free
        agent.acoustic_channels[channel_name] = AgentChannel()
        # randomized drift for each agent
        agent.acoustic_channels[channel_name].drift = self.rnd.uniform(-self._PPM, self._PPM) * 1e-6
        # randomize start time gap
        agent.acoustic_channels[channel_name].sync_gap = self.rnd.uniform(
            -self._sync_gap, self._sync_gap
            )
        # randomized computation time
        comp_bias = self.computational_delay + self.rnd.uniform(-self._e_delay[0], self._e_delay[0])
        e_delay = np.array([comp_bias, self._e_delay[1]])
        agent.acoustic_channels[channel_name].e_delay = e_delay
        # randomized biases range error
        e_range = np.array([self.rnd.uniform(
            -self.e_range[0], self.e_range[0]), self.e_range[1]
            ])
        agent.acoustic_channels[channel_name].e_range = e_range
        # randomized biases doppler error
        e_doppler = np.array([self.rnd.uniform(
            -self.e_doppler[0], self.e_doppler[0]), self.e_doppler[1]
            ])
        agent.acoustic_channels[channel_name].e_doppler = e_doppler


    def send (self, agent, msg_payload, msg_duration: float, collsion_avoidance: bool = True):
        """Send message from a given agent."""
        # refuse if another message is actively being transmitted
        if self.sim.time < agent.acoustic_channels[self.channel_name].release_time:
            out = "Refused - Already transmitting a message"
            self._sent_to_save.append((out, {'sender': agent.name}))
            return False, out
        # refuse if collision avoidance is active and reciving
        if collsion_avoidance:
            out = "Refused - Collision avoidance"
            if not agent.acoustic_channels[self.channel_name].status:
                self._sent_to_save.append((out, {'sender': agent.name}))
                return False, out
        
        # send message, use unique hash for each message key
        new_id = next(global_msg_id)
        self.active_msgs[new_id] = {
            'sender': agent.name,
            'start_loc': agent.pos,
            'front_radius': 0.0,
            'end_loc': None,
            'end_radius': None,
            'payload': msg_payload,
            'duration': msg_duration,
            'endtime': self.sim.time + msg_duration,
            'tod_raw': self.get_TimeOfDeparture(agent),
            'tod_exact': self.sim.time
        }

        # lock communication of agent until the message is fully sent
        agent.acoustic_channels[self.channel_name].release_time = self.sim.time + msg_duration
        self._sent_to_save.append(("Sent", self.active_msgs[new_id]))
        return True, "Sent"

    def get_TimeOfDeparture(self, agent):
        """Return the realistic (error affected) ToD for a message."""
        propriesties = agent.acoustic_channels[self.channel_name]
        # add drift over time and intial t0 difference
        tod = self.sim.time * (1 + propriesties.drift) + propriesties.sync_gap
        # add computation delay (fixed + random)
        tod = self.emulate_error(tod, propriesties.e_delay)
        return tod


    def __call__(self):
        """Resolve acoustic events."""
        # update all waves status
        result = {}
        self._evolve_waves()
        # check event for each wave (message)
        for id, msg in self.active_msgs.items():
            # check event for each agent 
            for name, agent in self.agents_dict.items():
                # exclude sender
                if msg['sender'] == name:
                    continue
                incoming = agent.acoustic_channels[self.channel_name].incoming
                # if new message id, verify if detected for the first time
                if not (id in incoming.id):
                    # if an event is detected_add it to the agent memory
                    event = self._wave_check(agent, msg['start_loc'], msg['front_radius'])
                    if event:
                        self._add_msg_to_agent(agent, id , msg)
                # for registered ids (front wave met), check if message is getting completed.
                else:
                    event = self._wave_check(agent, msg['end_loc'], msg['end_radius'])
                    if event:
                        # if it was the last id the the message is recived and read
                        if 1 == len(incoming.id):
                            result[name] = copy.deepcopy(self._return_msg_to_agent(agent, msg)) 
                        # remove id from id list
                        incoming.id.remove(id)
        self._event_to_save = result
        return result

    def _evolve_waves(self):
        # iterate all active message
        for id, msg in self.active_msgs.items():
            # advance front wave limited to MAX_RANGE
            msg['front_radius'] += self.C_SOUND * self.sim.Dt
            if msg['front_radius'] > self.MAX_RANGE:
                msg['front_radius'] = self.MAX_RANGE
            # check start end wave
            if msg['endtime'] > self.sim.time:
                continue
            # check if end wave is born in this step
            if msg['endtime'] > self.sim.time - self.sim.Dt:
                # set end wave initial position
                msg['end_loc'] = self.agents_dict[msg['sender']].pos
            # advance end wave
            msg['end_radius'] = (self.sim.time - msg['endtime']) * self.C_SOUND


    def _remove_waves(self):
        """Remove obsolete waves."""
        self.active_msgs = {
            k: v for k, v in self.active_msgs.items() if v['end_radius'] <= self.MAX_RANGE
            }


    def _wave_check(self, agent, center, radius):
        """Checks if a wave an agent has met a wave in the last timestep"""
        if center is None:
            return False
        radius_last_step = np.linalg.norm(agent.last_step_pos - center)
        radius_now = np.linalg.norm(agent.pos - center)
        was_outside = radius_last_step > radius - self.C_SOUND * self.sim.Dt
        is_inside =  radius_now <= radius
        return was_outside and is_inside


    def _add_msg_to_agent(self, agent, id, msg):
        """Add an incoming communication to an agent and resolve collisions.
        If only one message ID is present, the message is stored.
        If multiple IDs are present, this is treated as a collision.
        """
        channel = agent.acoustic_channels[self.channel_name]
        incoming = channel.incoming
        
        # Add message ID and lock the channel
        incoming.id.add(id)
        
        channel.status = False  

        if len(incoming.id) == 1:  
            # First/only entry → successful reception
            incoming.sender = msg["sender"]
            incoming.payload = msg["payload"]
            incoming.ToD_raw = msg["tod_raw"]
            incoming.ToD_exact = msg["tod_exact"]
            incoming.intact = True
            incoming.pos_at_detection = copy.deepcopy(agent.pos)
        else:  
            # Collision → invalidate message
            incoming.sender = None
            incoming.payload = None
            incoming.intact = False

    def _return_msg_to_agent(self, agent, msg):
        """ """
        channel = agent.acoustic_channels[self.channel_name]
        incoming = channel.incoming
        sender = self.sim.agents[msg['sender']]
        # Release Channel
        channel.status = True
        # positions at instant of front wave emission and reception
        sender_position = msg['start_loc']
        receiver_position = incoming.pos_at_detection  # with assumption of C_SOUND >> agent velocity
        # distance measurament  |  front wave instance 
        exact_distance = np.linalg.norm(sender_position - receiver_position)
        # distance at arrival
        incoming.perfect_range = exact_distance
        # continue calculation if the message is intact
        if not incoming.intact:
            return
        # Calculate ToA
        incoming.ToA_raw, incoming.ToA_exact = self.get_TimeOfArrival(channel, exact_distance, msg['end_radius'])
        # adding ranging noise (TwoWayTimeTravel-like measurament)
        incoming.ping_range = self.emulate_error(exact_distance, self.e_range)
        # get position at the endwave instance for doppler
        sender_endwave_position = self.sim.memory.recall_position(msg['endtime'], sender.name)
        receiver_endwave_position = agent.pos
        # calculate doppler value calculated: t0 time at front wave, t1 time at endwave
        #              t1 .     .      (p_r - p_s)                  ||p_r(t1) - p_s(t1)|| - ||p_r(t0) - p_s(t0)||
        # doppler =   ∫ (p_r - p_s) x ------------- dt  / (t1-t0)  = -----------------------------------------
        #              t0             ||p_r - p_s||                                   (t1 -t0)
        end_dist = np.linalg.norm(receiver_endwave_position - sender_endwave_position)
        doppler  = (end_dist - exact_distance) / msg["duration"]
        incoming.doppler_velocity = self.emulate_error(doppler, self.e_doppler)
        return incoming

    def get_TimeOfArrival(self, channel, distance, radius):
        """ """
        # calcualte exact time of arrival
        toa_exact = self.sim.time - (radius -  distance) / self.C_SOUND
        # add drift over time and intial t0 difference
        toa = toa_exact * (1 + channel.drift) + channel.sync_gap
        return toa, toa_exact


        
    def _bag(self):
        """Collect SQLite friendly output."""
        sent_rows = []
        timestep = self.sim.step_count

        # Process events
        for receiver, event in self._event_to_save.items():
            msg = asdict(event) if is_dataclass(event) else event
            # Handle pos_at_detection explicitly
            if msg["pos_at_detection"] is None:
                x, y, z = None, None, None
            else:
                arr = np.asarray(msg["pos_at_detection"]).flatten()
                x, y, z = arr.tolist()
            # append each event as a line
            sent_rows.append({
                "timestep": timestep,
                "status": "Received",
                "receiver": receiver,
                "sender": msg["sender"],
                "payload": msg["payload"],
                "intact": msg["intact"],
                "ToD_raw": msg["ToD_raw"],
                "ToD_exact": msg["ToD_exact"],
                "ToA_raw": msg["ToA_raw"],
                "ToA_exact": msg["ToA_exact"],
                "x_at_detection": x,
                "y_at_detection": y,
                "z_at_detection": z,
                "perfect_range": msg["perfect_range"],
                "ping_range": msg["ping_range"],
                "doppler_velocity": msg["doppler_velocity"],
                # special column for send event
                "duration": None,
                "x_start_loc": None,
                "y_start_loc": None,
                "z_start_loc": None,
            })

        # Process sending events if any
        if self._sent_to_save:
            for sent in self._sent_to_save:
                status, sent_data = sent
                
                sent_rows.append ({
                    "timestep": timestep,
                    # special column for send event
                    "status": status,
                    "duration": sent_data.get("duration"),
                    "x_start_loc": sent_data["start_loc"][0] if status == "Sent" else None,
                    "y_start_loc": sent_data["start_loc"][1] if status == "Sent" else None,
                    "z_start_loc": sent_data["start_loc"][2] if status == "Sent" else None,
                    # standard values and recivers
                    "receiver": None,
                    "sender": sent_data.get("sender"),
                    "payload": sent_data.get("payload") if status == "Sent" else None,
                    "intact": None,
                    "ToD_raw": sent_data.get("tod_raw") if status == "Sent" else None,
                    "ToD_exact": sent_data.get("tod_exact") if status == "Sent" else None,
                    "ToA_raw": None,
                    "ToA_exact": None,
                    "x_at_detection": None,
                    "y_at_detection": None,
                    "z_at_detection": None,
                    "perfect_range": None,
                    "ping_range": None,
                    "doppler_velocity": None,
                })


        # Empty the sending bag memory
        self._sent_to_save = []

        # Return SQLite-friendly list of dicts
        return sent_rows
    