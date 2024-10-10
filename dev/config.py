import os

from dotenv import load_dotenv

load_dotenv()
SENSOR_CONF_PORT = os.getenv("SENSOR_CONF_PORT", "COM8")
SENSOR_DATA_PORT = os.getenv("SENSOR_DATA_PORT", "COM9")
