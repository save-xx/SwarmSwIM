import numpy as np
import xml.etree.ElementTree as ET
import logging
import os

logger = logging.getLogger(__name__)


class MapSensor:
    def __init__(self, simulation, sensor_name, map_name):
        """
        
        """
        # path to map file (image type)
        folder = os.path.dirname(simulation._simulation_filepath)
        map_path = new_file = os.path.join(folder, map_name)
