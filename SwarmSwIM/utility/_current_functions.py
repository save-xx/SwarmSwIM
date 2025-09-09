import numpy as np


def parse_matrix(element):
    """Split the text into rows and then convert each row to a list of floats."""
    matrix = np.array([list(map(float, row.split())) for row in element.text.strip().split('\n')])
    return np.squeeze(matrix)


class VortexField:
    """Initiate vortex currents generator, time independent."""
    def __init__(self, density=30, intensity=0.5, size = 100, rng=np.random.default_rng()) -> None:
        # density is number of vortexes in a size square-meter area
        n_vortices = int(density) 
        self.SIZE = size
        # genrate vortexes and intensity on the random seed
        self.random_intensity = intensity*(2*rng.random(n_vortices)-1)
        self.vortex_centers = rng.uniform(0, self.SIZE, size=(n_vortices, 2))

    def single_vortex_contribution(self,x,y,vortex,intensity):
        """Calculates the current contribution of a single vortex."""
        xv, yv = vortex[0], vortex[1]
        # tiles the area, get the point nor furter than 50 on either axis
        if x-xv> self.SIZE/2: xv+=self.SIZE
        if x-xv<-self.SIZE/2: xv-=self.SIZE
        if y-yv> self.SIZE/2: yv+=self.SIZE
        if y-yv<-self.SIZE/2: yv-=self.SIZE
        # calculate intensity based on distance
        distance = (x-xv)**2+(y-yv)**2 
        vorticity = intensity / (distance + 1)**0.75
        # get vorticosity components 
        curr_x =   vorticity * (y-yv) 
        curr_y =  -vorticity * (x-xv) 
        return np.array([curr_x,curr_y])

    def current_vortex_calculate(self, agent):
        # module to remap position in the 0-100 aera
        x = agent.pos[0]%self.SIZE
        y = agent.pos[1]%self.SIZE
        # init total current
        current = np.array([0.0,0.0])
        # iterale every vortex and add up contribution
        for vortex, intensity in zip(self.vortex_centers,self.random_intensity):
            vortex_curr = self.single_vortex_contribution(x,y,vortex,intensity)
            current += vortex_curr
        return current


class TimeNoise:
    """Generate time based, space independent noise for each agent, with set frequency."""
    def __init__(self, time, freq=1.0,intensity=0.2,rng = None) -> None:
        self.rng = rng if rng is not None else np.random.default_rng()
        # set timer
        self.time = time
        self.t_min1 = time
        self.Tslot = 1/freq
        self.intensity = intensity

    def random_vector(self):
        """Generate a random vector."""
        mag = self.rng.uniform(0,1)*self.intensity
        ang = self.rng.uniform(0,2*np.pi)
        return np.array([mag*np.cos(ang),mag*np.sin(ang)])

    def init_noise(self):
        return np.array([self.random_vector(),self.random_vector()])
    
    def calculate_noises(self, now, agent):
        # initialize any missing agent.
        if not hasattr(agent, "current_noise"):
            setattr(agent, "current_noise", self.init_noise())
        # linear interpolate on time
        t = (now - self.time) / self.Tslot
        current = (1 - t) * agent.current_noise[0] + t * agent.current_noise[1]
        # do once when the sim time advances
        if self.t_min1 != now:
            self.t_min1 = now
            if now - self.time > self.Tslot:
                self.time = now
        # if self.time has just been updated then update current noise of the agent
        if now == self.time:
            prev_noise = agent.current_noise[1]
            agent.current_noise = np.array([prev_noise,self.random_vector()])

        return current


def calculate_global_waves(time_S , waves):
    """Generate a time dependant wave current."""
    # Formula: |v| = A*sin(wt+p)*versor(u)
    # S -> Reference to simulation, 
    # amplitude -> Module of velocity intensity A, 
    # frequency -> waves frequency w = 2pi*f, 
    # versor -> direction of output current expressed in [x,y]
    # shift -> time shift (for combined currents)

    # Unpacking
    total_current = np.array([0.,0.])
    for wave_param in waves:
        versor = wave_param['_versor']
        force = wave_param['amplitude'] * np.sin(wave_param['_w'] * time_S + wave_param['shift'])
        current = np.array([force * versor[0], force * versor[1]]).astype(float)
        total_current += current
    return total_current


def calculate_local_waves(time_S, agent, waves):
    """Generate a position and time dependant wave current."""
    # Unpacking
    total_current = np.array([0.,0.])
    for wave_param in waves:
        versor = wave_param['_versor']
        pos = agent.pos[0]*versor[0]+agent.pos[1]*versor[1]
        force = (wave_param['amplitude'] * 
                 np.sin(wave_param['_w'] * time_S + wave_param['_k']*pos + wave_param['shift']))
        current = np.array([force*versor[0], force*versor[1]])
        total_current += current
    return total_current