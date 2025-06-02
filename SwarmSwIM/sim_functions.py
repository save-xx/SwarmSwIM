import xml.etree.ElementTree as ET
import numpy as np
import os

DIR_FILE = os.path.dirname(__file__)
LOCAL_FILE = os.getcwd()
########### XML PARSING FUNCTIONS ###########

def parse_matrix(element):
# Split the text into rows and then convert each row to a list of floats
    matrix = np.array([list(map(float, row.split())) for row in element.text.strip().split('\n')])
    return np.squeeze(matrix)

def get_sim_xml_path(input_path):
    """Return absolute part of sim xml file"""
    if os.path.isabs(input_path): 
        path = input_path
    elif os.path.isfile(os.path.join(LOCAL_FILE, input_path)):
        path = os.path.join(LOCAL_FILE, input_path)
    else: 
        print ("No reference of sim xml found locally, using package directory")
        path = os.path.join(DIR_FILE, input_path)
    return path

def parse_envrioment_parameters(input_path):
    ''' unpack and return xml parameters for current settings'''
    data = {}
    data['global_waves']=[]
    data['local_waves']=[]
    path = get_sim_xml_path(input_path)

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

def parse_agents(input_path):
    ''' unpack and return parameters for adding agents'''
    data = {}
    path = get_sim_xml_path(input_path)
    tree = ET.parse(path)
    root = tree.getroot()
    agents_root = root.find('agents')
    if agents_root is not None and list(agents_root): 
        for agent_type in agents_root:
            filename = agent_type.find("description").text 
            dir_name = os.path.dirname(path)
            filename = os.path.join(dir_name,filename)
            nametype = agent_type.find("name").text 
            states = parse_matrix(agent_type.find('state'))
            # Handle single element case
            if 1==states.ndim: states=np.array([states])
            # Handle no element case
            if not states.size: continue 
            for i, state in enumerate(states):
                name = f"{nametype}{i+1:02}"
                data[name]=[state[0:3],state[3],filename]
    return data

########### CURRENT SIMULATION CLASSES AND FUNCTIONS ###########

class VortexField:
    ''' Vortex currents generator, time independent'''
    def __init__(self,density=30,intensity=0.5,rng=np.random.default_rng()):
        # density is number of vortexes in a 100 square-meter area
        n_vortices = int(density) 
        # genrate vortexes and intensity on the random seed
        self.random_intensity = intensity*(2*rng.random(n_vortices)-1)
        self.vortex_centers = rng.uniform(0, 100, size=(n_vortices, 2))

    def single_vortex_contribution(self,x,y,vortex,intensity):
        '''
        given a robot position in 0-100 area x,y and a vortex center,
          calculates the current contribution
        '''
        xv, yv = vortex[0], vortex[1]
        # tiles the area, get the point nor furter than 50 on either axis
        if x-xv> 50: xv+=100
        if x-xv<-50: xv-=100
        if y-yv> 50: yv+=100
        if y-yv<-50: yv-=100      
        # calculate intensity based on distance
        distance = (x-xv)**2+(y-yv)**2 
        vorticity = intensity / (distance + 1)**0.75
        # get vorticosity components 
        curr_x =   vorticity * (y-yv) 
        curr_y =  -vorticity * (x-xv) 
        return np.array([curr_x,curr_y])

    def current_vortex_calculate(self,agent):
        # module to remap position in the 0-100 aera
        x = agent.pos[0]%100
        y = agent.pos[1]%100
        # init total current
        current = np.array([0.0,0.0])
        # iterale every vortex and add up contribution
        for vortex, intensity in zip(self.vortex_centers,self.random_intensity):
            vortex_curr = self.single_vortex_contribution(x,y,vortex,intensity)
            current += vortex_curr
        return current

class TimeNoise:
    ''' generate time based, space independent noise for each agent, with set frequency'''
    def __init__(self,time,freq=1.0,intensity=0.2,rng = np.random.default_rng()) -> None:
        # seed for repetable random
        self.rng = rng
        # set timer
        self.time = time
        self.Tslot = 1/freq
        self.intensity = intensity
        # add each agent memory
        self.noises = {}

    def random_vector(self):
        ''' generate a random vector '''
        mag = self.rng.uniform(0,1)*self.intensity
        ang = self.rng.uniform(0,2*np.pi)
        return np.array([mag*np.cos(ang),mag*np.sin(ang)])

    def init_agent(self,agent):
        ''' add a new agent to the memory of noises'''
        self.noises[agent.name] = np.array([self.random_vector(),self.random_vector()])

    def throttle(self, now):
        if now - self.time <= self.Tslot: return
        # update timer
        self.time = now
        # update all existing noises 
        for key, item in self.noises.items():
            self.noises[key] = np.array([item[1], self.random_vector()])



    def calculate_noises(self,now,agent):
        # update all noises if needed
        self.throttle(now)
        # initialize any missing agent
        if not agent.name in self.noises:
            self.init_agent(agent)
        # linear interpolate on time
        t = (now-self.time)/self.Tslot
        current = (1-t)*self.noises[agent.name][0] + t*self.noises[agent.name][1]
        return current

def global_waves(time_S , amplitude=  0.2, frequency = 0.25 , direction = 0.0, shift = 0.0):
    ''' 
    Generate a time dependant wave current. Formula: |v| = A*sin(wt+p)*versor(u)
    S -> Reference to simulation, 
    amplitude -> Module of velocity intensity A, 
    frequency -> waves frequency w = 2pi*f, 
    versor -> direction of output current expressed in [x,y]
    shift -> time shift (for combined currents)
    '''
    w = 2*np.pi*frequency
    versor=[np.cos(np.deg2rad(direction)),np.sin(np.deg2rad(direction))]
    force = amplitude* np.sin(w*time_S+shift)
    current = np.array([force*versor[0], force*versor[1]]).astype(float)
    return current

def local_waves(time_S, agent, amplitude=  0.2, wavespeed = 0.5, wavelenght = 2 ,direction = 0.0, shift = 0.0):
    ''' generate a position and time dependant wave current'''
    if np.isclose(wavelenght, 0.0, atol=1e-9): return np.array([0.0,0.0])
    k = 2*np.pi/wavelenght
    w = k*wavespeed
    versor=[np.cos(np.deg2rad(direction)),np.sin(np.deg2rad(direction))]
    pos = agent.pos[0]*versor[0]+agent.pos[1]*versor[1]
    force = amplitude* np.sin(w*time_S + k*pos + shift)
    current = np.array([force*versor[0], force*versor[1]])
    return current

if __name__ == '__main__':
    data = parse_envrioment_parameters('simulation.xml')
    print(parse_agents('simulation.xml'))
