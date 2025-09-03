import numpy as np
from scipy.interpolate import RegularGridInterpolator
import xml.etree.ElementTree as ET
import logging
import imageio
import os

logger = logging.getLogger(__name__)




class MapSensor:
    def __init__(self, simulation, sensor_name, map_name, scale=1.0, method='linear'):
        """
        
        """
        self.sim = simulation
        self.sensor_name = sensor_name
        # path to map file (image type)
        folder = os.path.dirname(simulation._simulation_filepath)
        map_path = os.path.join(folder, map_name)
        self.scale = scale
        # load image and prepare interpolators
        self.img, gs = self.load_map(map_path)
        self.interpolators = self.create_interpolators(self.img, gs, method)
        # parse all agents
        for _ , agent in simulation.agents.items():
            self.parse_mapsensor(agent)

 
    @staticmethod
    def parse_mapsensor(self, agent, sensor_name):
        """ """
        # Initiate for all agents, regardless
        if not hasattr(agent, "sensor_map"):
            setattr(agent, "sensor_map", {})
        tree = ET.parse(agent._agent_filepath)
        root = tree.getroot()
        sensor_root = root.find(sensor_name)
        if sensor_root is None: 
            logger.warning("The XML does not contain a <sensors> element.")
            return
        detector_root = sensor_root.find(sensor_name)
        # the specific detector is not mounted, skip
        if detector_root is None:
            logger.warning(f"The XML does not contain a <{sensor_name}> element.")
            return
        # initiate sensor
        agent.sensor_map[sensor_name] = True


    def load_map(self, map_path):
        img = imageio.v2.imread(map_path)
        gs = False
        # grayscale → convert to RGB
        if img.ndim == 2:
            gs = True
            img = np.stack([img]*3, axis=-1)
        # drop alpha channel
        elif img.shape[-1] == 4:
            img = img[..., :3]
        # Fix orientation (transpose + flip)
        img = np.fliplr(np.transpose(img, (1, 0, 2)))
        return img, gs

    def create_interpolators(self, img, gs, method):
        h, w, c = img.shape
        # Create axis arrays for pixel centers
        x = (np.arange(w) - (w - 1) / 2.0) * self.scale
        y = (np.arange(h) - (h - 1) / 2.0) * self.scale
        # create interpolators
        interpolators = []
        for k in range(c):
            interp = RegularGridInterpolator(
                (y, x),  # (row=y, col=x)
                img[..., k],
                method=method,
                bounds_error=False,
                fill_value=0
            )
            interpolators.append(interp)
            if gs:
                break
        return interpolators

    def __call__(self) -> None:
        """ """
        result = {}
        for name , agent in self.sim.agents.items():
            # if agent has sensor
            if self.sensor_name in agent.sensor_map and agent.sensor_map[self.sensor_name]:
                # calculate value based on map
                result[name] = self.sample_map(self.interpolators, agent.pos)
        
        return result

    @staticmethod
    def sample_map(interpolators, pos):
        """
        Sample RGB/gray value at position.
        pos = (x, y) in world coordinates (centered at 0,0).
        """
        return np.array([interp((pos[1], pos[0])) for interp in interpolators])






        # # solve scale
        # if hasattr(scale, "__len__") and len(scale) == 1 and np.isscalar(scale[0]):
        #     scale = scale[0]
        # elif np.isscalar(scale):
        #     self.scale = np.array([scale, scale], dtype=float)
        # elif hasattr(scale, "__len__") and len(scale) == 2  and all(np.isscalar(x) for x in scale):
        #     self.scale = np.array(scale, dtype=float)
        # else:
        #     raise ValueError(f"MapSensor - Invalid scale: {scale}")

