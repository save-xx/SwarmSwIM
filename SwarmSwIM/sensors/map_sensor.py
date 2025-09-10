import numpy as np
from scipy.interpolate import RegularGridInterpolator
import xml.etree.ElementTree as ET
import logging
import imageio
import os

logger = logging.getLogger(__name__)


def activate_MapSensor(
    simulation,
    sensor_name: str,
    map_filename: str,
    scale: float = 1.0,
    method: str = 'linear'
):
    """
    Activate a map sensor plugin for a given simulation.

    This function creates a `MapSensor` instance for the specified sensor
    and map file, then registers it to the simulation so that it is called
    automatically at each post-step.

    Parameters
    ----------
    simulation : object
        The simulation instance containing agents and plugin infrastructure.
    sensor_name : str
        Unique name of the map sensor. This must correspond to the sensor
        tag in the agents' XML configuration.
    map_filename : str
        Path (relative to the simulation file) to the map image to be used
        by the sensor. It must inclue the file type (ex: .png, .jpg, ...)
    scale : float, optional
        Scaling factor for map coordinates. Default is 1.0. (one pixes per square-meter)
    method : str, optional
        Interpolation method for the map. Options include 'linear', 'nearest'.
        Default is 'linear'.

    Notes
    -----
    After activation, the sensor will be available in 
    `simulation.plugins_calls_poststep[sensor_name]` and invoked automatically
    after each simulation step. The sensor also initializes agent-specific
    flags based on their XML configuration.
    """
    # Create the MapSensor instance
    map_inst = MapSensor(simulation, sensor_name, map_filename, scale=scale, method=method)

    # Register the sensor in the simulation post-step call dictionary
    simulation.plugins_calls_poststep[sensor_name] = map_inst


class MapSensor:
    def __init__(self, simulation, sensor_name, map_name, scale=1.0, method='linear'):
        """
        
        """
        self.sim = simulation
        self.sensor_name = sensor_name
        self.last_result = {}
        # path to map file (image type)
        folder = os.path.dirname(simulation._simulation_filepath)
        map_path = os.path.join(folder, map_name)
        self.scale = scale
        # load image and prepare interpolators
        self.img, gs = self._load_map(map_path)
        self.interpolators, self.float16_interp = self._create_interpolators(self.img, gs, method)
        # parse all agents
        for _ , agent in simulation.agents.items():
            self.parse_mapsensor(agent, sensor_name)

 
    @staticmethod
    def parse_mapsensor(agent, sensor_name):
        """Parse check if the sensor is mounter for each agent"""
        # Initiate for all agents, regardless
        if not hasattr(agent, "sensor_map"):
            setattr(agent, "sensor_map", {})
        tree = ET.parse(agent._agent_filepath)
        root = tree.getroot()
        sensor_root = root.find("sensors")
        if sensor_root is None: 
            logger.warning("The XML does not contain a <sensors> element.")
            return
        detector_root = sensor_root.find(sensor_name)
        # the specific detector is not mounted, skip
        if detector_root is None:
            logger.info(f"{agent.name} does not mount <{sensor_name}>.")
            return
        # initiate sensor
        agent.sensor_map[sensor_name] = True


    def _load_map(self, map_path):
        img = imageio.v2.imread(map_path)
        gs = False
        # grayscale → convert to RGB
        if img.ndim == 2:
            gs = True
            img = np.stack([img]*3, axis=-1)
        # drop alpha channel
        elif img.shape[-1] == 4:
            img = img[..., :3]
        # Re-orient so map (0,0) is at center, x right, y up in NED
        img = np.fliplr(np.transpose(img, (1, 0, 2)))
        return img, gs

    def _create_interpolators(self, img, gs, method):
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

        # Float16 interpolator using R+G as packed float16
        # Compute float16 values from R+G
        # Stack last dimension
        rg_bytes = np.stack([img[..., 0], img[..., 1]], axis=-1)
        # View as float16
        packed_f16 = rg_bytes.view(np.float16).reshape(h, w)

        float16_interp = RegularGridInterpolator(
            (y, x),
            packed_f16,
            method=method,
            bounds_error=False,
            fill_value=0
        )

        return interpolators, float16_interp

    def __call__(self) -> None:
        """Vectorized sampling for all agents that mount this sensor."""
        active_agents = [
            (name, agent) for name, agent in self.sim.agents.items()
            if self.sensor_name in agent.sensor_map and agent.sensor_map[self.sensor_name]
        ]
        if not active_agents:
            self.last_result = {}
            return {}

        # Build query array: shape (N, 2) with (y, x) per agent
        positions = np.array([[agent.pos[1], agent.pos[0]] for _, agent in active_agents])

        if len(self.interpolators) == 1:
            # Grayscale → sample once, expand to RGB
            vals = self.interpolators[0](positions)  # shape (N,)
            channel_vals = np.repeat(vals[:, None], 3, axis=1)  # shape (N, 3)
        else:
            # RGB → sample all channels
            channel_vals = np.column_stack([interp(positions) for interp in self.interpolators])  # (N, 3)

        # Interpolate float16 value from R+G
        f16_vals = self.float16_interp(positions)  # N x 1

        # Combine channel values + float16
        values = np.column_stack([channel_vals, f16_vals])  # N x 4

        # Map back to agent names
        result = {name: values[i] for i, (name, _) in enumerate(active_agents)}

        self.last_result = result
        return result


    def _bag(self):
        """Bag collected data"""
        sent_rows = []
        timestep = self.sim.step_count
        for name, data in self.last_result.items():
            sent_rows.append({
                "timestep": int(timestep),
                "name": name,
                "channel_r": float(data[0]),
                "channel_g": float(data[1]),
                "channel_b": float(data[2]),
                "float16": float(data[3])
            })
        # Return SQLite-friendly list of dicts
        return sent_rows

# =================================================
# Utility functions to convert float images to RGB
# =================================================

def float_image_to_rgb(float_img: np.ndarray) -> np.ndarray:
    """
    Convert a 2D float image into a 3-channel RGB image.
    R and G store the float16 bytes, B is set to 0.
    """
    # Ensure input is float32/float64
    float_img = np.asarray(float_img, dtype=np.float32)
    
    # Convert each float to float16 bytes
    f16 = float_img.astype(np.float16)
    b = f16.tobytes()  # 2 bytes per value
    b = np.frombuffer(b, dtype=np.uint8)  # flat uint8 array
    
    # Reshape to H x W x 2
    H, W = float_img.shape
    b = b.reshape(H, W, 2)
    
    # Create RGB image
    rgb_img = np.zeros((H, W, 3), dtype=np.uint8)
    rgb_img[..., 0] = b[..., 0]  # R
    rgb_img[..., 1] = b[..., 1]  # G
    rgb_img[..., 2] = 0          # B
    
    return rgb_img

def convert_float_img_to_RGB(float_img: np.ndarray, filename: str) -> None:
    """
    Convert float image to RGB using float16 packing and save as lossless PNG.
    """
    rgb_img = float_image_to_rgb(float_img)
    imageio.imwrite(filename, rgb_img, format='PNG')