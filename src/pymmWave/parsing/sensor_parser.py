from abc import ABC, abstractmethod
from typing import Any, Dict

from serial import Serial


class SensorParser(ABC):
    """
    Base Sensor Parser Class. The goal of this class implementation is such that users can implement classes which can then be used with our library of algorithms easily.
    """

    @abstractmethod
    def parse(self, s: Serial) -> Dict | None:
        """Parse raw data from the sensor into a readable JSON-serializeable format. The sensor should be in a state where it already read the magic word, meaning the start of the packet will not contain the magic word.

        Args:
            s (Serial): Serial to read data from

        Returns:
            Dict | None: Parsed data, or None if the data is discarded
        """
        return {}
