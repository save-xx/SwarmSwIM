import xml.etree.ElementTree as ET
import numpy as np
import inspect
from numbers import Number
import logging
import os


logger = logging.getLogger(__name__)
DIR_FILE = os.path.dirname(__file__)
LOCAL_FILE = os.getcwd()

########### XML PARSING FUNCTIONS ###########

def xml_to_float(elem, default=0.0):
    """Turn single value tag to float"""
    if elem is None:
        return default
    if hasattr(elem, "text"):  # it's an XML element
        text = elem.text
    else:  # it's already a string
        text = elem
    if text is None or text.strip() == "":
        return default
    return float(text)


def parse_matrix(element):
    if element is None or element.text is None:
        return np.array([])
    # Split the text into rows and then convert each row to a list of floats
    matrix = np.array([list(map(float, row.split())) for row in element.text.strip().split('\n')])
    return np.squeeze(matrix)


def get_xml_path(input_path, default = "simulation.xml"):
    """Return absolute part of sim xml file"""
    # set default name is nothing is passed, or not a string
    if input_path == None or not isinstance(input_path, str): 
        input_path = default
        logger.debug("no name passed or invalid: using default name")
    if not input_path.endswith(".xml"):
        input_path += ".xml"
    # 1st choiche, if an absolute path is provided, priotize its use
    if os.path.isabs(input_path): 
        logger.debug("detected absolute path")
        return input_path
    # try to find the caller's script directory
    try:
        caller_file = inspect.stack()[2].filename
        running_dir = os.path.dirname(os.path.abspath(caller_file))
    except Exception:
        running_dir = ""
    # 2nd choiche: folder of the user defined script calling the Simulator class
    candidate = os.path.join(running_dir, input_path)
    logger.debug(f"User script folder: {running_dir}")
    if os.path.isfile(candidate):
        return candidate
    # 3rd choiche: for runtime execution, use the execution directory
    candidate = os.path.join(os.getcwd(), input_path)
    logger.debug(f"Execution directory: {os.getcwd()}")
    if os.path.isfile(candidate):
        return candidate
    # 4th choiche, use default defined simulation.xml
    logger.warning(f"No reference of {default} found, using default sim")
    return os.path.join(os.path.dirname(__file__), default)

def parse_agents(simulation_filepath):
    """Unpack and return Agents."""
    # initialize and open the file
    data = {}
    tree = ET.parse(simulation_filepath)
    root = tree.getroot()
    agents_root = root.find('agents')
    dir_name = os.path.dirname(simulation_filepath)
    # iterate agents types
    if agents_root is None or not list(agents_root):
        return data  # no agents to process

    for agent_type in agents_root:
        # generate name of description file, in same folder as simulation
        desc_tag = agent_type.find("description")
        name_tag = agent_type.find("name")
        # raise error, if the tag are absent from the description file
        if desc_tag is None or name_tag is None:
            raise ValueError("Missing <description> or <name> tag in agent block.")
        filename = os.path.join(dir_name, desc_tag.text)
        nametype = name_tag.text
        states = parse_matrix(agent_type.find('state'))
        # Handle single  and no element case
        if 1 == states.ndim: 
            states = np.array([states])
        if not states.size: 
            continue 
        for i, state in enumerate(states):
            # apply unique name
            name = f"{nametype}{i+1:02}"
            data[name] =[state[0:3], state[3], filename]
    return data

def get_seed(abs_path):
    """Extract seed value, if any."""
    tree = ET.parse(abs_path)
    root = tree.getroot()
    seed = int(root.find('seed').text) if root.find('seed') is not None else None
    return seed

def parse_envrioment_parameters(input_path):
    ''' unpack and return xml parameters for current settings'''
    data = {}
    data['global_waves']=[]
    data['local_waves']=[]
    path = get_xml_path(input_path)

    tree = ET.parse(path)
    root = tree.getroot()

    # get random seed if any
    data['seed'] = int(root.find('seed').text) if root.find('seed') is not None else None

    env_root = root.find('environment_setup')
    if env_root is None: raise ValueError("The XML does not contain a <environment_setup> element.")
    
    # uniform currents
    try: 
        uniform_current = parse_matrix(env_root.find('uniform_current'))
        if 0==uniform_current[0] and 0==uniform_current[1]: data['is_uniform_current']=False
        else:
            data['is_uniform_current']=True
            data['uniform_current'] = uniform_current
    except:
        data['is_uniform_current']=False

    # noise currets
    try: 
        noise_currents = parse_matrix(env_root.find('noise_currents'))
        if 0>=noise_currents[1]: data['is_noise_currents']=False
        else:
            data['is_noise_currents']=True
            data['noise_currents_freq'] = noise_currents[0]
            data['noise_currents_intensity'] = noise_currents[1]
    except:
        data['is_noise_currents']=False

    # vortex currents
    try: 
        vortex_currents = parse_matrix(env_root.find('vortex_currents'))
        if 0==int(vortex_currents[0]): data['is_vortex_currents']=False
        else:
            data['is_vortex_currents']=True
            data['vortex_currents_density'] = int(vortex_currents[0])
            data['vortex_currents_intensity'] = vortex_currents[1]
    except:
        data['is_vortex_currents']=False

    # global waves
    global_waves = env_root.find('global_waves')
    if global_waves is not None and list(global_waves): 
        data['is_global_waves']=True
        for wave in global_waves:
            wave_param={}
            wave_param['amplitude'] = float(wave.find("amplitude").text )
            wave_param['frequency'] = float(wave.find("frequency").text )
            wave_param['direction'] = float(wave.find("direction").text )
            wave_param['shift']     = float(wave.find("shift").text     )
            data['global_waves'].append(wave_param)

    else: data['is_global_waves']=False

    # local waves
    local_waves = env_root.find('local_waves')
    if local_waves is not None and list(local_waves): 
        data['is_local_waves']=True
        for wave in local_waves:
            wave_param={}
            wave_param['amplitude']  = float(wave.find("amplitude").text  )
            wave_param['wavelength'] = float(wave.find("wavelength").text )
            wave_param['wavespeed']  = float(wave.find("wavespeed").text  )
            wave_param['direction']  = float(wave.find("direction").text  )
            wave_param['shift']      = float(wave.find("shift").text      )
            data['local_waves'].append(wave_param)

    else: data['is_local_waves']=False

    # return structure with all unpacked data
    return data

###########

def generic_input(value):
    """
    Convert various input formats into a 2-element numpy float array.
    
    Accepted inputs:
      - Single number -> [number, 0.0]
      - List/tuple/array of length 1 -> [value, 0.0]
      - List/tuple/array of length 2 -> [value0, value1]
    
    Raises:
      ValueError if input is not numeric or length not in {1,2}.
    """
    # Convert numpy arrays to list for uniform handling
    if isinstance(value, np.ndarray):
        value = value.tolist()

    # Single number
    if isinstance(value, Number):
        return np.array([float(value), 0.0])

    # List/tuple of numbers
    if isinstance(value, (list, tuple)) and all(isinstance(x, Number) for x in value):
        if len(value) == 1:
            return np.array([float(value[0]), 0.0])
        elif len(value) == 2:
            return np.array(value, dtype=float)
        else:
            raise ValueError(
                f"Input must have length 1 or 2, got length {len(value)}"
            )

    raise ValueError(
        f"Unsupported input type {type(value)}. "
        "Must be a number, or a list/tuple/array of 1 or 2 numbers."
    )

########### CURRENT SIMULATION CLASSES AND FUNCTIONS ###########

# class VortexField:
#     ''' Vortex currents generator, time independent'''
#     def __init__(self,density=30,intensity=0.5,rng=np.random.default_rng()):
#         # density is number of vortexes in a 100 square-meter area
#         n_vortices = int(density) 
#         # genrate vortexes and intensity on the random seed
#         self.random_intensity = intensity*(2*rng.random(n_vortices)-1)
#         self.vortex_centers = rng.uniform(0, 100, size=(n_vortices, 2))

#     def single_vortex_contribution(self,x,y,vortex,intensity):
#         '''
#         given a robot position in 0-100 area x,y and a vortex center,
#           calculates the current contribution
#         '''
#         xv, yv = vortex[0], vortex[1]
#         # tiles the area, get the point nor furter than 50 on either axis
#         if x-xv> 50: xv+=100
#         if x-xv<-50: xv-=100
#         if y-yv> 50: yv+=100
#         if y-yv<-50: yv-=100      
#         # calculate intensity based on distance
#         distance = (x-xv)**2+(y-yv)**2 
#         vorticity = intensity / (distance + 1)**0.75
#         # get vorticosity components 
#         curr_x =   vorticity * (y-yv) 
#         curr_y =  -vorticity * (x-xv) 
#         return np.array([curr_x,curr_y])

#     def current_vortex_calculate(self,agent):
#         # module to remap position in the 0-100 aera
#         x = agent.pos[0]%100
#         y = agent.pos[1]%100
#         # init total current
#         current = np.array([0.0,0.0])
#         # iterale every vortex and add up contribution
#         for vortex, intensity in zip(self.vortex_centers,self.random_intensity):
#             vortex_curr = self.single_vortex_contribution(x,y,vortex,intensity)
#             current += vortex_curr
#         return current

# class TimeNoise:
#     ''' generate time based, space independent noise for each agent, with set frequency'''
#     def __init__(self,time,freq=1.0,intensity=0.2,rng = np.random.default_rng()) -> None:
#         # seed for repetable random
#         self.rng = rng
#         # set timer
#         self.time = time
#         self.Tslot = 1/freq
#         self.intensity = intensity
#         # add each agent memory
#         self.noises = {}

#     def random_vector(self):
#         ''' generate a random vector '''
#         mag = self.rng.uniform(0,1)*self.intensity
#         ang = self.rng.uniform(0,2*np.pi)
#         return np.array([mag*np.cos(ang),mag*np.sin(ang)])

#     def init_agent(self,agent):
#         ''' add a new agent to the memory of noises'''
#         self.noises[agent.name] = np.array([self.random_vector(),self.random_vector()])

#     def throttle(self, now):
#         if now - self.time <= self.Tslot: return
#         # update timer
#         self.time = now
#         # update all existing noises 
#         for key, item in self.noises.items():
#             self.noises[key] = np.array([item[1], self.random_vector()])



#     def calculate_noises(self,now,agent):
#         # update all noises if needed
#         self.throttle(now)
#         # initialize any missing agent
#         if not agent.name in self.noises:
#             self.init_agent(agent)
#         # linear interpolate on time
#         t = (now-self.time)/self.Tslot
#         current = (1-t)*self.noises[agent.name][0] + t*self.noises[agent.name][1]
#         return current

# def global_waves(time_S , amplitude=  0.2, frequency = 0.25 , direction = 0.0, shift = 0.0):
#     ''' 
#     Generate a time dependant wave current. Formula: |v| = A*sin(wt+p)*versor(u)
#     S -> Reference to simulation, 
#     amplitude -> Module of velocity intensity A, 
#     frequency -> waves frequency w = 2pi*f, 
#     versor -> direction of output current expressed in [x,y]
#     shift -> time shift (for combined currents)
#     '''
#     w = 2*np.pi*frequency
#     versor=[np.cos(np.deg2rad(direction)),np.sin(np.deg2rad(direction))]
#     force = amplitude* np.sin(w*time_S+shift)
#     current = np.array([force*versor[0], force*versor[1]]).astype(float)
#     return current

# def local_waves(time_S, agent, amplitude=  0.2, wavespeed = 0.5, wavelenght = 2 ,direction = 0.0, shift = 0.0):
#     ''' generate a position and time dependant wave current'''
#     if np.isclose(wavelenght, 0.0, atol=1e-9): return np.array([0.0,0.0])
#     k = 2*np.pi/wavelenght
#     w = k*wavespeed
#     versor=[np.cos(np.deg2rad(direction)),np.sin(np.deg2rad(direction))]
#     pos = agent.pos[0]*versor[0]+agent.pos[1]*versor[1]
#     force = amplitude* np.sin(w*time_S + k*pos + shift)
#     current = np.array([force*versor[0], force*versor[1]])
#     return current

# if __name__ == '__main__':
#     data = parse_envrioment_parameters('simulation.xml')
#     print(parse_agents('simulation.xml'))
