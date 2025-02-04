import os

import numpy as np
from dotenv import load_dotenv

load_dotenv()
SENSOR_CONF_PORT = os.getenv("SENSOR_CONF_PORT", "COM8")
SENSOR_DATA_PORT = os.getenv("SENSOR_DATA_PORT", "COM9")
SENSOR_ELEVATION_TILT = float(
    np.radians(float(os.getenv("SENSOR_ELEVATION_TILT", -10)))
)
SENSOR_HEIGHT = float(os.getenv("SENSOR_HEIGHT", 1.5))
