from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, Optional

from scipy.spatial.transform.rotation import Rotation

from .logging import Logger, StdOutLogger


class Sensor(ABC):
    """
    Base sensor class. The goal of this class implementation is such that users can implement classes which can then be used with our library of algorithms easily.
    """

    def __init__(self) -> None:
        super().__init__()
        self._log: Logger = StdOutLogger()

    def set_logger(self, new_logger: Logger):
        """Replace the default stdout logger with another.

        Args:
            new_logger (Logger): The logger to use. Must implement Logger base class.
        """
        self._logger = new_logger

    def log(self, *args: Any, **kwargs: Any) -> None:
        """Log something to the logger."""
        self._log.log(*args, **kwargs)

    def error(self, *args: Any, **kwargs: Any) -> None:
        """Report an error to the logger."""
        self._log.log(*args, **kwargs)

    @abstractmethod
    def model(self) -> str:
        """Return the model of a sensor

        Returns:
            str: Name of sensor
        """
        pass

    @abstractmethod
    def is_alive(self) -> bool:
        """Check if sensor is still alive

        Returns:
            bool: True if alive
        """
        pass

    @abstractmethod
    async def start_sensor(self) -> None:
        """Asynchronous loop that can be run as a coroutine with asyncio, or other asynchronous libraries.

        Returns:
            Nothing, will be run as a coroutine!
        """
        pass

    @abstractmethod
    def stop_sensor(self):
        """Stop sensor

        Returns:
            Nothing, kills everything
        """
        pass

    @abstractmethod
    async def get_data(self) -> Dict:
        """Return data from sensor."""
        pass

    @abstractmethod
    def get_data_nowait(self) -> Optional[Dict]:
        """Return data from sensor if available, otherwise None.

        Returns:
            Optional[DataModel]: Data if there is data available, otherwise returns None.
        """
        pass

    @abstractmethod
    def get_update_freq(self) -> float:
        """Returns the sensor update freq. This is reccommended to be the actual rate of data access."""
        pass


class SpatialSensor(object):
    """Wrapper to provide the notion of a sensor in space"""

    def __init__(
        self,
        sens: Sensor,
        location: tuple[float, float, float],
        pitch_rads: tuple[float, float, float],
    ):
        self.sensor = sens
        self.location = location

        # This speeds up code later
        self.pitch_rads: Rotation = Rotation.from_rotvec(pitch_rads)  # type: ignore


class InvalidSensorException(Exception):
    def __init__(self, message: str, errors: str):
        super().__init__(message)
