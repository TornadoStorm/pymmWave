from abc import ABC, abstractmethod
from typing import Any, Dict

from serial import Serial


class SensorParser(ABC):
    """
    Base Sensor Parser Class
    """

    @abstractmethod
    def parse(self, s: Serial) -> Dict:
        """
        Parse the data from the sensor.
        """
        return {}
