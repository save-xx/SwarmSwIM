"""This contains current and disturbances emulation"""


import xml.etree.ElementTree as ET
import numpy as np

from SwarmSwIM.utility._current_functions import parse_matrix
from SwarmSwIM.utility._current_functions import VortexField, TimeNoise
from SwarmSwIM.utility._current_functions import calculate_global_waves, calculate_local_waves


def activate_Currents (simulation):
    """
    Activate the ocean current plugin for a simulation.

    This function adds a current field to the simulation, allowing agents 
    to experience environmental flow effects (e.g., water currents). 
    The plugin is automatically called before each simulation step.

    Parameters
    ----------
    simulation : object
        The simulation instance to which the current plugin will be added.

    Notes
    -----
    - The current plugin instance is stored as `simulation.currents`.
    - It is registered to `simulation.plugins_calls_prestep`, so it will be
      executed automatically before each simulation step.
    """
    current_inst = Currents(simulation)
    setattr(simulation, 'currents', current_inst)
    simulation.plugins_calls_prestep["currents"] = simulation.currents


class Currents:
    def __init__(self, simulation):
        # internal reference to simulation
        self.simulation = simulation
        # check if the data exist in the xml
        # return the root of data
        envrioment_root = self.check_env_root(simulation._simulation_filepath)
        # Unpack all currents
        self.is_uniform, self.uniform = self.unpack_uniform(envrioment_root)
        self.is_noise, self.noise = self.unpack_noise(envrioment_root)
        self.is_vortex, self.vortex = self.unpack_vortex(envrioment_root)
        self.is_global_waves, self.global_waves = self.unpack_global_waves(envrioment_root)
        self.is_local_waves, self.local_waves = self.unpack_local_waves(envrioment_root)
        # Initialize class based currents
        if self.is_vortex:
            self.set_vortex(rnd=simulation.rnd)
        if self.is_noise:
            self.set_noise(rnd=simulation.rnd)

    def __call__(self):
        """Calcualte and apply current disturbances."""
        # calcualte effect of agent indepentent current effects
        # global waves
        global_waves_vector = calculate_global_waves(
            self.simulation.time,
            self.global_waves
            )
        # calculate agent dependent current effects, for each agent
        for name, agent in self.simulation.agents.items():
            if not hasattr(agent, 'use_currents') or not agent.use_currents:
                continue
            agent_current_vector = np.array([0.,0.])

            # use all current (including time dependent)
            if agent.use_currents == 'all':
                # Noise
                if self.is_noise:
                    agent_current_vector += self._noise_inst.calculate_noises(
                        self.simulation.time,
                        agent
                    )
                # Global Waves
                if self.is_global_waves:
                    agent_current_vector += global_waves_vector
                
                # Local Waves
                if self.is_local_waves:
                    agent_current_vector += calculate_local_waves(
                        self.simulation.time,
                        agent,
                        self.local_waves
                    )

            # add time independent contributions
            if self.is_vortex:
                agent_current_vector += self._vortex_inst.current_vortex_calculate(agent)

            # uniform current
            if self.is_uniform:
                agent_current_vector += self.uniform
            
            # apply current to agent
            agent.pos += np.append(agent_current_vector, 0) * self.simulation.Dt

    # -------------------------------------------
    # Parsing and Unpacking methods and functions
    # -------------------------------------------
    @staticmethod
    def check_env_root(path):
        """Verify that the simulation file contains the envrioment tag."""
        tree = ET.parse(path)
        root = tree.getroot()
        env_root = root.find('environment_setup')
        if env_root is None: 
            raise ValueError("The XML does not contain a <environment_setup> element.")
        return env_root
    
    @staticmethod
    def unpack_uniform(env_root):
        """Unpack uniform currents."""
        uniform = np.array([0.,0.])
        is_uniform = False
        try: 
            uniform_current = parse_matrix(env_root.find('uniform_current'))
            if 0 != uniform_current[0] or 0 != uniform_current[1]: 
                uniform = uniform_current
                is_uniform = True
        finally:
            pass
        return is_uniform, uniform
    
    @staticmethod
    def unpack_noise(env_root):
        """Unpack noise currents."""
        noise = {'hz': None, 'intensity': None}
        is_noise = False
        try: 
            noise_current = env_root.find('noise_currents')
            noise['hz'] = float(noise_current.find('frequency').text)
            noise['intensity'] = float(noise_current.find('intensity').text)
            # both frequency and intensity must be greater than 0
            if 0 < noise['hz'] and 0 < noise['intensity']: 
                is_noise = True
        finally:
            pass
        return is_noise, noise
    
    @staticmethod
    def unpack_vortex(env_root):
        """Unpack large votrexes currents."""
        vortex = {'density': None, 'intensity': None, 'size': 100}
        is_vortex = False
        try: 
            vortex_current = env_root.find('vortex_currents')
            vortex['size'] = float(vortex_current.find('size').text)
            vortex['density'] = int(vortex_current.find('density').text)
            vortex['intensity'] = float(vortex_current.find('intensity').text)
            # both size, frequency and intensity must be greater than 0
            if 0 < vortex['size'] and 0 < vortex['density'] and 0 < vortex['intensity']: 
                is_vortex = True
        finally:
            pass
        return is_vortex, vortex

    def unpack_global_waves(self, env_root):
        """Unpack global waves currents."""
        is_global_waves = False
        global_waves = []

        try: 
            global_waves_currents = env_root.find('global_waves')
        except:
            global_waves_currents = None

        if global_waves_currents is not None and list(global_waves_currents): 
            for wave in global_waves_currents:
                amplitude = float(wave.find("amplitude").text)
                frequency = float(wave.find("frequency").text)
                direction = float(wave.find("direction").text)
                shift = float(wave.find("shift").text)
                if amplitude == 0:
                    continue
                # create wave parameters dictionary
                wave_param = self._add_global_wave(amplitude, frequency, direction, shift)
                global_waves.append(wave_param)
            # set wave as true
            is_global_waves = True
        return is_global_waves, global_waves
    

    def unpack_local_waves(self, env_root):
        """Unpack local waves currents."""
        is_local_waves = False
        local_waves = []

        try: 
            local_waves_currents = env_root.find('local_waves')
        except:
            local_waves_currents = None

        if local_waves_currents is not None and list(local_waves_currents): 
            for wave in local_waves_currents:
                amplitude = float(wave.find("amplitude").text)
                wavelength = float(wave.find("wavelength").text)
                wavespeed = float(wave.find("wavespeed").text)
                direction = float(wave.find("direction").text)
                shift = float(wave.find("shift").text)
                if np.isclose(wavelength, 0.0, atol=1e-9) or amplitude == 0:
                    continue
                # create wave parameters dictionary
                wave_param = self._add_local_wave(amplitude, wavelength, wavespeed, direction, shift)
                local_waves.append(wave_param)
            # set wave as true
            is_local_waves = True
        return is_local_waves, local_waves

    # --------------------------
    # Setting Currents functions
    # --------------------------
    def set_vortex(self, **kwargs):
        """Initialize a vortexes instance."""
        density = kwargs['density'] if 'density' in kwargs else self.vortex['density']
        intensity = kwargs['intensity'] if 'intensity' in kwargs else self.vortex['intensity']
        size = kwargs['size'] if 'size' in kwargs else self.vortex['size']
        # initiate class
        self._vortex_inst = VortexField(
            density=density, 
            intensity=intensity, 
            rng=self.simulation.rnd
            )
        # activate vortexes
        self.is_vortex = True

    def set_noise(self, **kwargs):
        """Initialize a noise instance"""
        hz = kwargs['hz'] if 'hz' in kwargs else self.noise['hz']
        intensity = kwargs['intensity'] if 'intensity' in kwargs else self.noise['intensity']
        # initiate class
        self._noise_inst = TimeNoise(
            self.simulation.time, 
            freq=hz, 
            intensity=intensity, 
            rng=self.simulation.rnd
            )
        # activate vortexes
        self.is_noise = True

    def _add_global_wave(self, amplitude, frequency, direction=0.0, shift=0.0):
        wave_param = {}
        wave_param['amplitude'] = amplitude
        wave_param['frequency'] = frequency
        wave_param['direction'] = direction
        wave_param['shift'] = shift
        # internal parameters
        wave_param['_w'] = 2 * np.pi * wave_param['frequency']
        wave_param['_versor'] = [np.cos(np.deg2rad(wave_param['direction'])),
                                 np.sin(np.deg2rad(wave_param['direction']))]
        # return dict
        return wave_param

    def add_global_wave(self, amplitude, frequency, direction=0.0, shift=0.0):
        """Add a new global wave"""
        wave_param = self._add_global_wave(amplitude, frequency, direction, shift)
        # add to preexisting list
        self.global_waves.append(wave_param)

    def _add_local_wave(self, amplitude, wavelength, wavespeed, direction=0.0, shift=0.0):
        wave_param = {}
        wave_param['amplitude'] = amplitude
        wave_param['wavelength'] = wavelength
        wave_param['wavespeed'] = wavespeed
        wave_param['direction'] = direction
        wave_param['shift'] = shift
        # internal parameters
        wave_param['_k'] = 2*np.pi / wavelength
        wave_param['_w'] = wave_param['_k'] * wavespeed
        wave_param['_versor'] = [np.cos(np.deg2rad(wave_param['direction'])),
                                 np.sin(np.deg2rad(wave_param['direction']))]
        # return dict
        return wave_param

    def add_local_wave(self, amplitude, wavelength, wavespeed, direction=0.0, shift=0.0):
        """Add a new global wave"""
        wave_param = self._add_local_wave(self, amplitude, wavelength, wavespeed, direction, shift)
        # add to preexisting list
        self.global_waves.append(wave_param)

# -------
# Testing
# -------
if __name__ == "__main__":
        
    class agent_dummy:
        def __init__(self):
            self.name = 'A'
            self.pos = np.array([0.,0.,0.])
            self.use_currents = None

    class sim_dummy:
        def __init__(self):
            self._simulation_filepath = 'simulation.xml'
            self.agents = {'A': agent_dummy()}
            self.Dt = 0.1
            self.time = 1.0

    sim = sim_dummy()
    c = Currents(sim)
    print (c.uniform)
    c()