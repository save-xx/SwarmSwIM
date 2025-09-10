"""File defining Agent Class."""
from . import sim_functions
from .sim_functions import parse_matrix, xml_to_float, generic_input
import numpy as np
import xml.etree.ElementTree as ET
import os
import logging
import inspect


DIR_FILE = os.path.dirname(__file__)
LOCAL_FILE = os.getcwd()
logger = logging.getLogger(__name__)

class Agent():
    def __init__(self, name,
                 initialPosition=np.array([0.0, 0.0, 0.0]),
                 initialHeading=0.0, agent_xml="default.xml", rng=None):
        """
        Agent Object: parameters.

        - name: (str) unique name
        - Dt: (float) time subdivision
        - initialPosition: (array-like size 3) global position at start. Default [0,0,0]
        - initialHeading: float: initial heading in degrees, default is 0.0 (North)
        - agent_xml: filename of agent setting, default is default.xml
        - rng: (int) random seed value
        """
        # private
        self._cmd_force = np.array([0.0, 0.0])
        self._cmd_local_vel = np.array([0.0, 0.0])
        self._cmd_planar = np.array([0.0, 0.0])
        # Fixed time division
        self.Dt = 0.1 # < placeholder overwritten by simulator
        # Set inital condition
        self.name = name

        # Set initial position
        if not isinstance(initialPosition, (list, tuple, np.ndarray)) or len(initialPosition) != 3:
            raise ValueError("Initial position must be a list, tuple, or numpy array of length 3.")
        # Convert to float numpy array
        self.pos = np.array(initialPosition, dtype=float)

        # Set initial heading
        self.psi = initialHeading
        # Genrate random seed based on name
        if not rng:
            self.rnd = np.random.default_rng()
        else:
            self.rnd = np.random.default_rng(hash(name) % 2**20 + rng)

        # Load agent parameters from xml
        self.agent_type = agent_xml
        self._agent_filepath = sim_functions.get_xml_path(agent_xml)
        self._parse_agent_parameters()
        # Parameter initialization
        self.incurrent_velocity = np.array([0, 0])
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
        self.cmd_local_vel = np.array([0, 0])
        self.cmd_forces = np.array([0, 0])
        # step memory
        self.last_step_pos = self.pos.copy()
        self.other_forces = np.array([0, 0])

    def __repr__(self):
        # NOTE: by design, the name of each agent is enforced to be unique.
        return (f"Agent<{self.name}>")

    @property
    def cmd_forces(self):
        return self._cmd_force

    @cmd_forces.setter
    def cmd_forces(self, input):
        self._cmd_force = generic_input(input)

    @property
    def cmd_local_vel(self):
        return self._cmd_local_vel

    @cmd_local_vel.setter
    def cmd_local_vel(self, input):
        self._cmd_local_vel = generic_input(input)

    @property
    def cmd_planar(self):
        return self._cmd_planar

    @cmd_planar.setter
    def _cmd_planar(self, input):
        if len(input) != 2:
            raise ValueError(f"cmd_planar input must be of length 2: passed {input}")
        self._cmd_local_vel = np.array(input, dtype=float)

    # ================================

    def _parse_agent_parameters(self):
        """Read the xml file for the agents charateristics."""
        # Utility
        # --------------------
        # Local Function - parsing of vectors and matrix from XML

        def read_2d_parameters(name):
            """
            Create an attiribute based on the input string name.

            Populate with corresponding values in xml if any
            For parameters of 2 elements.
            """
            if sim_agent.find(name) is not None:
                setattr(self, name, parse_matrix(sim_agent.find(name)))
            else:
                setattr(self, name, np.zeros(2))

        # --------------------

        path = self._agent_filepath
        tree = ET.parse(path)
        root = tree.getroot()
        sim_agent = root.find('sim_agent')
        if sim_agent is None:
            raise ValueError("The XML does not contain a <sim_agent> element.")
        # --------------------
        # Parse mass parameters
        self.mass = xml_to_float(sim_agent.find('mass'), 1.0)
        
        if sim_agent.find('dimentions') is not None:
            self.dimentions = parse_matrix(sim_agent.find('dimentions'))
        else:
            self.dimentions = np.array([0.4, 0.2, 0.3])

        if sim_agent.find('added_mass') is not None:
            self.added_mass = parse_matrix(sim_agent.find('added_mass'))
        else:
            self.added_mass = np.zeros([2, 2])
        self.tot_mass = self.added_mass + np.array([[self.mass, 0], [0, self.mass]])

        # selection of control scheme [REQUIRED]
        self.depth_control = sim_agent.find('depth_control').text
        self.heading_control = sim_agent.find('heading_control').text
        self.planar_control = sim_agent.find('planar_control').text
        
        # Parse damping parameters
        if sim_agent.find('linear_damping') is not None: 
            self.linear_damping = parse_matrix(sim_agent.find('linear_damping'))
        else:
            self.linear_damping = np.zeros([2, 2])
        if sim_agent.find('quadratic_damping') is not None:
            self.quadratic_damping = parse_matrix(sim_agent.find('quadratic_damping'))
        else: 
            self.quadratic_damping = np.identity(2)
        
        # Parse Control depth parameters
        self.step_depth = xml_to_float(sim_agent.find("step_depth"), default=0.0)
        self.proportional_depth = xml_to_float(sim_agent.find("proportional_depth"), default=0.0)
        self.heave_limit = xml_to_float(sim_agent.find("heave_limit"), default=0.0)
        # Parse Control heading
        self.step_heading = xml_to_float(sim_agent.find("step_heading"), default=0.0)
        self.proportional_heading = xml_to_float(sim_agent.find("proportional_heading"), default=0.0)
        self.yawrate_limit = xml_to_float(sim_agent.find("yawrate_limit"), default=0.0)
        # Parse Control planar
        self.step_planar = xml_to_float(sim_agent.find("step_planar"), default=0.0)
        read_2d_parameters("vel_limit")
        self.yawrate_limit = xml_to_float(sim_agent.find("yawrate_limit"), default=0.0)

        # List of the names to be added in the noise parameters
        list_of_noises = ['e_depth', 'e_heave', 'e_heading', 'e_yawrate',
                          'e_position', 'e_local_vel', 'e_inertial_vel',
                          'e_local_force']
        
        # parse navigational noises
        for name in list_of_noises:
            read_2d_parameters(name)
        
        # currents
        use_currents = sim_agent.find("use_currents")
        no_current  = (use_currents is None 
                        or use_currents.text is None 
                        or use_currents.text.strip().lower() in ("", "0", "false"))
        if no_current:
            self.use_currents = False
        else:
             self.use_currents = use_currents.text

    def emulate_error(self, data, error):
        """Alter the input data to simulate measurment errors."""
        data += error[0]
        data += self.rnd.normal(scale=error[1])
        return data

    def tick(self):
        """
        Update the Agent.
        Advances the simulation forward one step for the agent.
        """
        self._update_feedback_sensors()
        self._update_heading()
        self._update_depth()
        self._update_planar(self.Dt)


    def _update_feedback_sensors(self):
        """
        Update value of control feedback sensor.
        Specifically affects the sensors involved in emulating the control scheme applied to the agent.
        """
        self.measured_depth = self.emulate_error(self.pos[2], self.e_depth)
        self.measured_heading = (self.emulate_error(
            self.psi, self.e_heading)) % 360
        self.measured_pos = np.array([
            self.emulate_error(self.pos[0], self.e_position),
            self.emulate_error(self.pos[1], self.e_position)
            ])

    def _update_depth(self):
        """Update Agent depth, based on selected behavior."""
        correction = self.cmd_depth - self.measured_depth
        if "ideal" == self.depth_control:
            self.pos[2] += correction
        elif "step" == self.depth_control:
            step = self.Dt*self.step_depth
            if abs(correction) < step:
                self.pos[2] += correction
            else:
                self.pos[2] = self.pos[2] + step * np.sign(correction)
        elif "proportional" == self.depth_control:
            w = self.proportional_depth * correction
            if w > self.heave_limit:
                w = self.heave_limit
            if w < -self.heave_limit:
                w = -self.heave_limit
            self.pos[2] += self.emulate_error(w, self.e_heave)*self.Dt
        elif "heave" == self.depth_control:
            self.pos[2] += self.emulate_error(self.cmd_heave, self.e_heave) * self.Dt

    def _update_heading(self):
        """Update Agent heading, based on selected behavior."""
        correction = (self.cmd_heading - self.measured_heading) % 360
        if correction > 180:
            correction -= 360

        if "ideal" == self.heading_control:
            self.psi += correction

        elif "step" == self.heading_control:
            step = (self.Dt*self.step_heading)
            if abs(correction) < step:
                self.psi += correction
            else:
                self.psi += step * np.sign(correction)

        elif "proportional" == self.heading_control:
            r = self.proportional_heading * correction
            if r > self.yawrate_limit:
                r = self.yawrate_limit
            if r < -self.yawrate_limit:
                r = -self.yawrate_limit
            self.psi += self.emulate_error(r, self.e_yawrate)*self.Dt

        elif "yawrate" == self.heading_control:
            self.psi += self.emulate_error(self.cmd_yawrate, self.e_yawrate) * self.Dt
        # return result in the [0,360) range
        self.psi %= 360

    def _update_planar(self, Dt):
        """Tick based update of the planar position."""
        def get_emulated_velocities():
            return np.array([self.emulate_error(self.cmd_local_vel[0], self.e_local_vel),
                    self.emulate_error(-self.cmd_local_vel[1], self.e_local_vel)])

        def get_emulated_inertial():
            return np.array([self.emulate_error(self.cmd_local_vel[0], self.e_inertial_vel),
                    self.emulate_error(-self.cmd_local_vel[1], self.e_inertial_vel)])
        
        # calculate correction and angles
        x_correction = self.cmd_planar[0]-self.measured_pos[0]
        y_correction = self.cmd_planar[1]-self.measured_pos[1]
        sinpsi = np.sin(np.deg2rad(self.psi))
        cospsi = np.cos(np.deg2rad(self.psi))
        R_mat = np.array(((cospsi,sinpsi),(sinpsi,-cospsi)))

        if "ideal" == self.planar_control:
            self.pos[0] += x_correction
            self.pos[1] += y_correction

        elif "step" == self.planar_control:
            step = (self.Dt*self.step_planar)
            d_correction = np.linalg.norm([x_correction, y_correction])
            if abs(d_correction) < step:
                self.pos[0] += x_correction
                self.pos[1] += y_correction
            else:
                self.pos[0] += step*x_correction/d_correction
                self.pos[1] += step*y_correction/d_correction

        elif "local_velocity" == self.planar_control:
            emulated_velocities = get_emulated_velocities()
            self.pos[0] += (emulated_velocities[0] * cospsi + emulated_velocities[1] * sinpsi) * Dt
            self.pos[1] += (emulated_velocities[0] * sinpsi - emulated_velocities[1] * cospsi) * Dt

        elif "inertial_velocity" == self.planar_control:
            step = (self.Dt * self.vel_limit)
            emulated_velocities = get_emulated_inertial()
            # current effect in the last step, in body axis
            current_disturbance = self.pos[0:2] - self.last_step_pos[0:2] 
            current_disturbance_body = R_mat.transpose() @ current_disturbance
            # current_disturbance_body[1] *= -1
            # real tranlation on the plane required by the controller, body frame
            real_translation = emulated_velocities * self.Dt - current_disturbance_body
            # thresholding based on velocity limit
            if abs(real_translation[0]) - step[0] > 0:
                real_translation[0] = np.copysign(step[0], real_translation[0])
            if abs(real_translation[1]) - step[1] > 0:
                real_translation[1] = np.copysign(step[1], real_translation[1])
            # apply thresholded velocity correction
            self.pos[0] += (real_translation[0] * cospsi + real_translation[1] * sinpsi)
            self.pos[1] += (real_translation[0] * sinpsi - real_translation[1] * cospsi)

        elif "local_forces" == self.planar_control:
            # NED convention
            # TODO Add effective Forces and e_forces
            real_actuation = [self.emulate_error(self.cmd_forces[0], self.e_local_force),
                              self.emulate_error(-self.cmd_forces[1], self.e_local_force)]
            external_forces = np.array([real_actuation[0]+self.other_forces[0],
                                        -real_actuation[1]-self.other_forces[1]])
            F_tot = (external_forces +
                     np.matmul(self.linear_damping, self.incurrent_velocity) +
                     np.matmul(self.quadratic_damping, abs(self.incurrent_velocity)*self.incurrent_velocity)
                     )
            acc_local = np.matmul(np.linalg.inv(self.tot_mass), F_tot)
            self.incurrent_velocity = self.incurrent_velocity + acc_local * Dt
            Dx = (self.incurrent_velocity[0] * cospsi + self.incurrent_velocity[1] * sinpsi) * Dt
            Dy = (self.incurrent_velocity[0] * sinpsi - self.incurrent_velocity[1] * cospsi) * Dt
            self.pos[0] += Dx
            self.pos[1] += Dy

        # Save last step state
        self.last_step_pos = self.pos.copy()


    # =========================
    # Built-in command packages 
    # =========================


    def set_ForceHeadingDepth(self, 
                              forceNewton: float | list | np.ndarray | None = None, 
                              headingDegrees: float | None = None, 
                              depthMeters: float | None = None
                              ):
        """
        Control in Planar force, step heading and depth.

        - forceNewton: (array-like size 2 or float)
            planar force vector in Newton, referred to body frame
          - if float, assume force vector in the direction of heading (surge).
        - headingDegrees: (float)
            planar direction (psi) in degrees. NED convention
        - depthMeters: (float)
            desired depth in meters
        """
        if forceNewton is not None:
            self.cmd_forces = forceNewton
        if headingDegrees is not None:
            self.cmd_heading = headingDegrees % 360
        if depthMeters is not None:
            self.cmd_depth = depthMeters


    def set_PositionHeading(self, 
                            positionMeters: list | np.ndarray, 
                            headingDegrees: float | None = None
                            ):
        """
        Position command,  planar step control, heading and depth.

        - positionMeters: (array-like size 2 or 3)
            position vector respect to global coordinates.
          - If len==3 depth is updated, otherwise previous depth is mantained.
        - headingDegrees: (float, Optional),
            heading of the agent.
        """
        # Set desired planar Coordinates
        self.cmd_planar = positionMeters[0:2]
        if 3 == len(positionMeters):
            self.cmd_depth = positionMeters[3]
        if headingDegrees is not None:
            self.cmd_heading = headingDegrees


    def set_ForceCmd(self, 
                     forceNewton: float | list | np.ndarray,
                     enforce: bool = False
                     ):
        """
        Set force command.
        - forceNewton: (array-like size 2 or float)
            planar force vector in Newton, referred to body frame.
            If a single value is provvided, it is referred to the x-axis.
        - enforce: (bool, default `False`)
            enforce planar control to `local_forces` mode.
        """
        self.cmd_forces = forceNewton
        if enforce:
            self.planar_control = "local_forces"


    def set_VelocityCmd(self, 
                        velocity: float | list | np.ndarray,
                        mode: str | None = None
                        ):
        """
        Set velocity command.
        - velocity: (array-like size 2 or float)
            planar velocity vector in Newton, referred to body frame.
            If a single value is provvided, it is referred to the x-axis.
        - mode: (str or None, default `None`)
            if indicated, set planar control to the specified mode.
        """
        self.cmd_local_vel = velocity
        if not mode:
            return
        if not mode in ("local_velocity", "inertial_velocity"):
            raise ValueError(f"mode must be either `local_velocity` or `inertial_velocity`. Input {mode}")
        self.planar_control = mode


    def set_WaypointCmd(self, 
                        waypoint: float | list | np.ndarray,
                        mode: str | None = None
                        ):
        """
        Set a planar position command (waypoint).
        - waypoint: (array-like size 2)
            planar velocity vector in Newton, referred to body frame.
        - mode: (str or None, default `None`)
            if indicated, set planar control to the specified mode.
        """
        self.cmd_planar = waypoint
        if not mode:
            return
        if not mode in ("ideal", "step"):
            raise ValueError(f"mode must be either `ideal` or `step`. Input {mode}")
        self.planar_control = mode


    def set_Heading(self, 
                    headingDegrees: float,
                    mode: str | None = None
                    ):
        """
        Set heading command.
        - headingDegrees: float
            heading, expressed in degrees, NED conventions (0 is North)
        - mode: (str or None, default `None`)
            if indicated, set planar control to the specified mode.
        """
        self.cmd_heading = headingDegrees % 360
        if not mode:
            return
        if not mode in ("ideal", "step", "proportional"):
            raise ValueError(f"mode must be either `ideal`,  `step` or `proportional`. Input {mode}")
        self.heading_control = mode


    def set_Yawrate(self, 
                    yawrate: float,
                    enforce: bool = False
                    ):
        """
        Set yawrate command.
        - yawrate: float
            yawrate, expressed in degrees per second, NED conventions.
        - enforce: (bool, default `False`)
            if indicated, set planar control to the specified mode.
        """
        self.cmd_yawrate = yawrate
        if enforce:
            self.heading_control = "yawrate"


    def set_Depth(self, 
                    depthMeters: float,
                    mode: str | None = None
                    ):
        """
        Set depth command.
        - depthMeters: float
            depth, expressed in meters, NED conventions (positive is down).
        - mode: (str or None, default `None`)
            if indicated, set planar control to the specified mode.
        """
        self.cmd_depth = depthMeters
        if not mode:
            return
        if not mode in ("ideal", "step", "proportional"):
            raise ValueError(f"mode must be either `ideal`,  `step` or `proportional`. Input {mode}")
        self.depth_control = mode


    def set_Heave(self, 
                    heave: float,
                    enforce: bool = False
                    ):
        """
        Set heave (descent/ascend speed) command.
        - heave: float
            yawrheaveate, expressed in meters per second, NED conventions (positive is down).
        - mode: (bool, default `False`)
            if indicated, set planar control to the specified mode.
        """
        self.cmd_heave = heave
        if enforce:
            self.depth_control = "heave"
    


if __name__ == '__main__':
    A1 = Agent("A1", 0.1)
    A1.planar_control = "inertial_velocity"
    A1.cmd_forces = np.array([1, 0])
    A1.cmd_depth = 0.0
    A1.cmd_heading = 0
    print(A1.mass)
    print(A1.e_inertial_vel)
    for i in range(30):
        A1.tick()
        if i%10==0: print(f'{A1.pos[0]:.6f}, {A1.psi:.6f}')