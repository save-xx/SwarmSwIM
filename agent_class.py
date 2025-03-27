import numpy as np
import xml.etree.ElementTree as ET
import os

DIR_FILE = os.path.dirname(__file__)

class Agent():
    def __init__(self, name, Dt=0.1, initialPosition = np.array([0.0,0.0,0.0]), initialHeading = 0.0, agent_xml="default.xml",rng=None):
        '''
        Agent Object: parameters  
        - name: (str) unique name   
        - Dt: (float) time subdivision  
        - initialPosition: (array-like size 3) determining global position at start. Default [0,0,0]  
        - initialHeading: float: initial heading in degrees, default is 0.0 (North)  
        - agent_xml: filename of agent setting, default is default.xml  
        - rng: (int) random seed value
        '''
        # private
        self._cmd_force = np.array([0.0,0.0])
        self._cmd_local_vel = np.array([0.0,0.0])
        # Fixed time division
        self.Dt = Dt
        # Set inital condition
        self.name = name
        # Set initial positiion
        if not 3==len(initialPosition): raise ValueError ('Passed initial position is not a 3 element Array or list')
        if type(initialPosition) in (tuple,list): self.pos =np.array(initialPosition).astype(float) #convert to numpy array
        else: self.pos = initialPosition.astype(float)
        # Set initial heading
        self.psi = initialHeading
        # Genrate random seed based on name
        if not rng: self.rnd = np.random.default_rng()
        else: self.rnd = np.random.default_rng(hash(name)%2**20 + rng)
        # Load agent parameters from xml
        self.parse_agent_parameters(agent_xml)
        # Parameter initialization
        self.internal_clock= 0.0
        self.incurrent_velocity = np.array([0,0])
        # Sensors initialization
        self.measured_depth = initialPosition[2]
        self.measured_heading = initialHeading
        self.measured_pos = initialPosition[0:2]
        # Command initialization
        self.cmd_depth = initialPosition[2]
        self.cmd_heave = 0
        self.cmd_heading = initialHeading
        self.cmd_yawrate = 0
        self.cmd_planar = np.array(initialPosition[0:2])
        self.cmd_local_vel = np.array([0,0])
        self.cmd_forces = np.array([0,0])
        # step memory 
        self.last_step_planar = np.array(initialPosition[0:2])
        self.other_forces = np.array([0,0])

    @property
    def cmd_forces(self):
        return self._cmd_force
    @cmd_forces.setter
    def cmd_forces(self,input):
        self._cmd_force = generic_input(input)

    @property
    def cmd_local_vel(self):
        return self._cmd_local_vel
    @cmd_local_vel.setter
    def cmd_local_vel(self,input):
        self._cmd_local_vel = generic_input(input)

    def parse_agent_parameters(self, local_path):
        ''' Read the xml file for the agents charateristics'''
        # Utility
        #--------------------
        # Local Function - parsing of vectors and matrix from XML
        def parse_matrix(element):
            ''' Split the text into rows and then convert each row to a list of floats '''
            matrix = np.array([list(map(float, row.split())) for row in element.text.strip().split('\n')])
            return np.squeeze(matrix)
        
        def read_float_parameter(name,default_value):
            ''' Given a string, create an attiribute with that name an populate'''
            if sim_agent.find(name) is not None: # if found in the file, populate the variable
                setattr(self,name,float(sim_agent.find(name).text))
            else: setattr(self,name,float(default_value))

        def read_2d_parameters(name):
            ''' Given a string, create an attiribute with that name, 
            and populate with corresponding values in xml if any
            For parameters of 2 elements'''
            if sim_agent.find(name) is not None: # if found in the file, populate the variable
                setattr(self,name,parse_matrix(sim_agent.find(name)))
            else: setattr(self,name,np.zeros(2))
        #--------------------
        # fix path if local or global
        if os.path.isabs(local_path): path = local_path
        else: path = os.path.join(DIR_FILE,local_path)
        tree = ET.parse(path)
        root = tree.getroot()
        sim_agent = root.find('sim_agent')
        if sim_agent is None: raise ValueError("The XML does not contain a <sim_agent> element.")
        #--------------------
        # Parse mass parameters
        read_float_parameter('mass',1.0)
        if sim_agent.find('added_mass') is not None: self.added_mass = parse_matrix(sim_agent.find('added_mass'))
        else: self.added_mass = np.zeros([2,2])
        self.tot_mass = self.added_mass + np.array([[self.mass,0],[0,self.mass]])
        # selection of control scheme [REQUIRED]
        self.depth_control = sim_agent.find('depth_control').text
        self.heading_control = sim_agent.find('heading_control').text
        self.planar_control = sim_agent.find('planar_control').text
        # Parse damping parameters < at least one is required >
        if sim_agent.find('linear_damping') is not None: self.linear_damping = parse_matrix(sim_agent.find('linear_damping'))  
        else: self.linear_damping = np.zeros([2,2])
        if sim_agent.find('quadratic_damping') is not None:  self.quadratic_damping = parse_matrix(sim_agent.find('quadratic_damping'))
        else: self.quadratic_damping = np.identity(2)
        # Parse Control depth parameters
        read_float_parameter('step_depth',0.0)
        read_float_parameter('proportional_depth',0.0)
        read_float_parameter('heave_limit',0.0)
        # Parse Control heading
        read_float_parameter('step_heading',0.0)
        read_float_parameter('proportional_heading',0.0)
        read_float_parameter('yawrate_limit',0.0)
        # Parse Control planar
        read_float_parameter('step_planar',0.0)
        read_2d_parameters('vel_limit')
        read_float_parameter('yawrate_limit',0.0)

        # List of the names to be added in the noise parameters
        list_of_noises = ['e_depth','e_heave','e_heading','e_yawrate',
                          'e_position','e_local_vel','e_inertial_vel',
                          'e_local_force']
        # parse navigational noises
        for name in list_of_noises:
            read_2d_parameters(name)
        # additional clock drift error
        self.clock_drift = float(sim_agent.find('clock_drift').text) \
                                if sim_agent.find('clock_drift') is not None \
                                else 0
        ## TODO consider adding randomization
        #--------------------
        # Parse Sensors <- TODO consider delocating
        sensors_root = root.find('sensors')
        self.sensors={}
        if not (sensors_root is None):      #skip if not defined
        # NN Detector
            detector_root = sensors_root.find('NNDetector')
            if detector_root:
                self.sensors['NNDetector']={'period': float(detector_root.find('period').text)}
                self.sensors['NNDetector']['field_of_view']= parse_matrix(detector_root.find('field_of_view'))
                self.sensors['NNDetector']['visibility_model'] = detector_root.find('visibility_model').text
                if "linear"== self.sensors['NNDetector']['visibility_model']:
                    self.sensors['NNDetector']['points'] = parse_matrix(detector_root.find('points'))
                #Parse Sensor errors
                self.sensors['e_NND_distance'] = parse_matrix(detector_root.find('e_distance')) if detector_root.find('e_distance') is not None else np.zeros(2)
                self.sensors['e_NND_alpha'] =    parse_matrix(detector_root.find('e_alpha'))    if detector_root.find('e_alpha')    is not None else np.zeros(2)
                self.sensors['e_NND_beta'] =    parse_matrix(detector_root.find('e_beta'))      if detector_root.find('e_beta')     is not None else np.zeros(2)
                # add detector element
                self.NNDetector={'time_lapsed': self.rnd.uniform(0,self.sensors['NNDetector']['period'])}
            acoustic_root = sensors_root.find('Acoustic_Ranging')
            if acoustic_root:
                self.sensors['ac_msg_length'] = float(acoustic_root.find('msg_length').text)
                self.sensors['e_ac_range'] = parse_matrix(acoustic_root.find('e_ac_range')) if acoustic_root.find('e_ac_range') is not None else np.zeros(2)
                self.sensors['e_ac_doppler']  = parse_matrix(acoustic_root.find('e_doppler'))  if acoustic_root.find('e_doppler')  is not None else np.zeros(2)


    def emulate_error (self, data, error):
        ''' Alter the input data to simulate measurment errors '''
        data += error[0]
        data += self.rnd.normal(scale=error[1])
        return data

    def tick(self):
        ''' Update the Agent tick'''
        self.update_internal_clock()
        self.update_sensors()
        self.update_heading()
        self.update_depth()
        self.update_planar(self.Dt)
        
    def update_internal_clock(self):
        ''' Keep track of the internal time clock'''
        self.internal_clock += self.Dt*(1+self.clock_drift*1e-6)

    def update_sensors(self):
        ''' Update value of control feedback sensor'''
        self.measured_depth = self.emulate_error(self.pos[2],self.e_depth)
        self.measured_heading = (self.emulate_error(self.psi,self.e_heading)) %360
        self.measured_pos = np.array([
            self.emulate_error(self.pos[0],self.e_position),
            self.emulate_error(self.pos[1],self.e_position) ])

    def update_depth(self):
        ''' Update Agent depth, based on selected behavior '''
        correction = self.cmd_depth - self.measured_depth
        if   "ideal"==self.depth_control:
            self.pos[2] += correction
        elif "step"==self.depth_control:
            step = self.Dt*self.step_depth
            if abs(correction)< step: self.pos[2] += correction
            else: 
                self.pos[2] = self.pos[2] + step * np.sign(correction)
        elif "proportional"==self.depth_control:
            w = self.proportional_depth* correction
            if w> self.heave_limit: w = self.heave_limit
            if w<-self.heave_limit: w =-self.heave_limit
            self.pos[2] += self.emulate_error(w,self.e_heave)*self.Dt
        elif "heave"==self.depth_control:
            self.pos[2] += self.emulate_error(self.cmd_heave,self.e_heave)*self.Dt

    def update_heading(self):
        ''' Update Agent heading, based on selected behavior '''
        correction = (self.cmd_heading - self.measured_heading)%360
        if correction>180: correction -=360
        if "ideal"==self.heading_control:
            self.psi += correction
        elif "step"==self.heading_control:
            step = (self.Dt*self.step_heading)
            if abs(correction)<step: self.psi += correction
            else: self.psi += step * np.sign(correction)
        elif "proportional"==self.heading_control:
            r = self.proportional_heading* correction
            if r> self.yawrate_limit: r = self.yawrate_limit
            if r<-self.yawrate_limit: r =-self.yawrate_limit
            self.psi += self.emulate_error(r,self.e_yawrate)*self.Dt
        elif "yawrate"==self.heading_control:
            self.psi += self.emulate_error(self.cmd_yawrate,self.e_yawrate)*self.Dt
        # return result in the [0,360) range
        self.psi %=360

    def update_planar(self, Dt):
        ''' tick based update of the planar position'''
        def get_emulated_velocities():
            return [self.emulate_error( self.cmd_local_vel[0],self.e_local_vel),
                    self.emulate_error(-self.cmd_local_vel[1],self.e_local_vel)]
        def get_emulated_inertial():
            return [self.emulate_error( self.cmd_local_vel[0],self.e_inertial_vel),
                    self.emulate_error(-self.cmd_local_vel[1],self.e_inertial_vel)]
        ## calculate correction and angles
        x_correction = self.cmd_planar[0]-self.measured_pos[0]
        y_correction = self.cmd_planar[1]-self.measured_pos[1]
        sinpsi = np.sin(np.deg2rad(self.psi))
        cospsi = np.cos(np.deg2rad(self.psi)) 

        if "ideal"==self.planar_control:
            self.pos[0] += x_correction
            self.pos[1] += y_correction
        elif "step"==self.planar_control:
            step = (self.Dt*self.step_planar)
            d_correction = np.linalg.norm([x_correction,y_correction])
            if abs(d_correction)<step: 
                self.pos[0] += x_correction
                self.pos[1] += y_correction
            else:
                self.pos[0] += step*x_correction/d_correction
                self.pos[1] += step*y_correction/d_correction
        elif "local_velocity"==self.planar_control:
            emulated_velocities = get_emulated_velocities()
            self.pos[0] += (emulated_velocities[0] * cospsi + emulated_velocities[1] * sinpsi) * Dt
            self.pos[1] += (emulated_velocities[0] * sinpsi - emulated_velocities[1] * cospsi) * Dt
        elif "inertial_velocity"==self.planar_control:
            step = (self.Dt*self.vel_limit)
            emulated_velocities = get_emulated_inertial() 
            ideal_x_pos = self.last_step_planar[0] + (emulated_velocities[0] * cospsi + emulated_velocities[1] * sinpsi) * Dt
            ideal_y_pos = self.last_step_planar[1] + (emulated_velocities[0] * sinpsi - emulated_velocities[1] * cospsi) * Dt
            # limitation of maximal velocity vector
            if abs(ideal_x_pos-self.pos[0])<step[0]: self.pos[0] = ideal_x_pos
            else: self.pos[0] += np.copysign(step[0], ideal_x_pos-self.pos[0])
            if abs(ideal_y_pos-self.pos[1])<step[1]: self.pos[1] = ideal_y_pos
            else: self.pos[1] += np.copysign(step[1], ideal_y_pos-self.pos[1])

        elif "local_forces"==self.planar_control:
            # NED convention
            # TODO Add effective Forces and e_forces
            real_actuation = [self.emulate_error( self.cmd_forces[0],self.e_local_force),
                              self.emulate_error(-self.cmd_forces[1],self.e_local_force)]
            external_forces = np.array([ real_actuation[0]+self.other_forces[0],
                                        -real_actuation[1]-self.other_forces[1]])
            F_tot = (external_forces +
                     np.matmul(self.linear_damping, self.incurrent_velocity)+
                     np.matmul(self.quadratic_damping,abs(self.incurrent_velocity)*self.incurrent_velocity)
                     )
            acc_local = np.matmul(np.linalg.inv(self.tot_mass),F_tot)
            self.incurrent_velocity = self.incurrent_velocity + acc_local * Dt
            Dx = (self.incurrent_velocity[0] * cospsi + self.incurrent_velocity[1] * sinpsi) * Dt
            Dy = (self.incurrent_velocity[0] * sinpsi - self.incurrent_velocity[1] * cospsi) * Dt
            self.pos[0] += Dx
            self.pos[1] += Dy

        ## Save last step state
        self.last_step_planar = self.pos[0:2].copy()

    ## Built-in command packages ##

    def cmd_fhd(self, forceNewton ,headingDegrees ,depthMeters):
        ''' 
        Control in Planar force, step heading and depth  
        - forceNewton: (array-like size 2 or float) planar force vector in Newton, referred to body frame
          - if float, assume force vector in the direction of heading (surge).
        - headingDegrees: (float) planar direction (psi) in ddegrees. NED convention
        - depthMeters: (float) desired depth in meters
        '''
        self.cmd_forces = forceNewton
        self.cmd_heading = headingDegrees
        self.cmd_depth = depthMeters

    def cmd_xyz_phi (self, positionMeters, headingDegrees=None):
        ''' 
        Position command,  planar step control, heading and depth  
        - positionMeters: (array-like size 2 or 3) position vector respect to global coordinates.  
          - If of size 3 depth is updated, otherwise previous depth is mantained.  
        - headingDegrees: (float, Optional), heading of the agent.  
        '''
        # Set desired planar Coordinates
        self.cmd_planar = positionMeters[0:2]
        if 3==len(positionMeters): self.cmd_depth = positionMeters[3]
        if headingDegrees: self.cmd_heading = headingDegrees

NUM_TYPES = (int, float, np.integer, np.floating) 
def generic_input(input):
    ''' returns a 2 element np.array of a generic input'''
    # Handle np.arrays as inputs
    if   isinstance(input,np.ndarray): 
        if 2==np.size(input): return input.squeeze().astype('float')
        elif 1 == np.size(input): return np.array([float(input),0.0])
        else: raise ValueError(f'Input is of not accetable size, allowed 1 or 2, current {np.size(input)}')
    # Handle lists
    elif isinstance(input, (list, tuple)) and all(isinstance(x, NUM_TYPES) for x in input):
        if len(input) == 2: return np.array(input).astype('float')
        elif len(input) == 1: return np.array([float(input[0]),0.0])
        else: raise ValueError(f'Input is of not accetable size, allowed 1 or 2, current {len(input)}')
    # Handle single numbers
    elif isinstance(input, NUM_TYPES):
        return np.array([float(input),0.0])
    else: raise ValueError('Input type or length incorrect')

if __name__ == '__main__':
    A1 = Agent("A1",0.1)
    A1.cmd_forces = np.array([1,0])
    A1.cmd_depth = 0.0
    A1.cmd_heading = 0
    print(A1.mass)
    print(A1.e_inertial_vel)
    for i in range(30):
        A1.tick()
        # if i%10==0: print(f'{A1.pos[0]:.6f}, {A1.psi:.6f}')
