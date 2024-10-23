from asyncio import Queue, sleep
from dataclasses import dataclass
from time import time
from typing import Dict, Optional

from serial import Serial  # type: ignore
from serial.serialutil import SerialException

from .constants import ASYNC_SLEEP, MAGIC_NUMBER
from .parsing.area_scanner.area_scanner_parser import AreaScannerParser
from .parsing.sensor_parser import SensorParser
from .sensor import Sensor


@dataclass(init=False)
class Frame:
    """Frame class for the sensor. Only holds data attributes, similar to a struct"""

    packet: float
    idxPacket: float
    header: float
    detObj: float
    rp: float
    np: float
    tlv_version: bytes
    tlv_version_uint16: int
    tlv_platform: int
    frameNumber: int
    numDetectedObj: int = -1
    detectedPoints_byteVecIdx: int = -1


class IWR6843AOP(Sensor):
    """Abstract :obj:`Sensor<mmWave.sensor.Sensor>` class implementation for interfacing with the COTS TI IWR6843AOP evaluation board.
    Can be initialized with a public 'name', which can be used for sensor reference.
    """

    parser: SensorParser
    """The parser used to parse raw data from the sensor. Defaults to AreaScannerParser()"""

    def __init__(self, name: str, verbose: bool = False):
        """Initialize the sensor

        Args:
            verbose (bool, optional): Print out extra initialization information, can be useful. Defaults to False.
        """

        super().__init__()
        self._is_alive: bool = False
        self._ser_config: Optional[Serial] = None
        self._ser_data: Optional[Serial] = None
        self._verbose = verbose
        self._config_sent = False
        self.name = name
        self._config_port_name: Optional[str] = None
        self._data_port_name: Optional[str] = None
        self._config_baud: Optional[int] = None
        self._data_baud: Optional[int] = None

        # Why a queue? This is forward looking. asyncio defaults to single threaded behavior and therefore this should
        #   be thread safe by default. The upside of a queue is if this changes to a multi-process system on some executor,
        #   this code remains valid as this is a safe shared option.
        self._active_data: Queue[dict] = Queue(1)
        self._freq: float = 10.0
        self._last_t: float = 0.0

        self.parser: SensorParser = AreaScannerParser()

    def connect_config(self, com_port: str, baud_rate: int, timeout: int = 1) -> bool:
        """Connect to the config port. Must be done before sending config.
        This function will timeout after a second by default. This timeout period is low since programmatically connecting to serial ports might be difficult with long timeout periods, as it is difficult to know apriori if you are connecting to config or data.

        Args:
            com_port (str): Port name to use
            baud_rate (int): Baud rate
            timeout (int, optional): Timeout. Defaults to 1.

        Returns:
            bool: True if successful

        Example:
            On MacOS, for example:

            >>> my_sensor.connect_config('/dev/tty.SLAB_USBtoUART4', 115200)
            True

        """
        try:
            self._ser_config = Serial(com_port, baud_rate, timeout=timeout)
        except SerialException as e:
            print(e)
            return False
        except FileNotFoundError:
            print(f"{com_port} is an invalid serial port.")
            return False
        except ValueError:
            print("Baud rate is invalid.")
            return False

        self._update_alive()
        self._config_port_name = com_port
        self._config_baud = baud_rate

        return True

    def connect_data(self, com_port: str, baud_rate: int, timeout: int = 1) -> bool:
        """Connect the data serial port. Must be done before sending config.

        Args:
            com_port (str): Port name to use
            baud_rate (int): Baud rate
            timeout (int, optional): Timeout. Defaults to 1.

        Returns:
            bool: True if successful

        Example:
            On MacOS, for example:

            >>> my_sensor.connect_data('/dev/tty.SLAB_USBtoUART', 921600)
            True
        """

        try:
            self._ser_data = Serial(com_port, baud_rate, timeout=timeout)
        except SerialException as e:
            print(e)
            return False
        except FileNotFoundError:
            print(f"{com_port} is an invalid serial port.")
            return False
        except ValueError:
            print("Baud rate is invalid.")
            return False

        self._update_alive()
        self._data_port_name = com_port
        self._data_baud = baud_rate

        return True

    def _update_alive(self):
        """Internal func to verify the sensor is still connected"""
        if self._ser_config is not None and self._ser_data is not None:  # type: ignore
            self._is_alive = self._ser_config.is_open and self._ser_data.is_open  # type: ignore

        if not self._is_alive:
            self._config_sent = False

    def is_alive(self) -> bool:
        """Getter which verifies connection to this particular sensor.

        Returns:
            bool: True if the sensor is still connected.
        """

        self._update_alive()
        return self._is_alive

    def model(self) -> str:
        """Returns the particular model number supported with this class.

        Returns:
            str: "IWR6843AOP"
        """
        return "IWR6843AOP"

    def send_config(
        self, config: list[str], max_retries: int = 1, autoretry_cfg_data: bool = True
    ) -> bool:
        """Tried to send a TI config, with a simple retry mechanism.
        Configuration files can be created here: `https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.5.0/`. Future support may be built for creating configuration files.

        Args:
            config (list[str]): List of strings making up the config
            max_retries (int, optional): Number of times to retry on failure. Defaults to 1.

        Returns:
            bool: If sending was successful

        Raises:
            SerialException: If device is disconnected before completion, SerialExceptions may be raised.
        """
        valid_replies = set(["Done", "Ignored: Sensor is already stopped"])
        if not self._is_alive:
            self._config_sent = False
            return False

        attempts = 0
        failed = False
        while attempts < max_retries:
            attempts += 1
            failed = False
            for line in config:
                if line[0] == "%" or line[0] == "\n":
                    pass
                else:
                    ln = line.replace("\r", "")
                    self._ser_config.write(ln.encode())  # type: ignore

                    try:
                        ret1 = self._ser_config.readline()  # type: ignore
                        ret1 = ret1.decode("utf-8")[:-1].strip()  # type: ignore
                    except UnicodeDecodeError:
                        ret1 = "error"

                    try:
                        ret = self._ser_config.readline()  # type: ignore
                        ret = ret.decode("utf-8")[:-1].strip()  # type: ignore
                    except UnicodeDecodeError:
                        ret = "error"

                    # We look at both replies just in case of an error in ordering of results. Rare, but can happen.
                    if not ((ret1 in valid_replies) or (ret in valid_replies)):
                        failed = True
                        self.log("invalid replies:", ret1, ret)
                        self.error("Sending configuration failed!")
                        break
                    # if self._verbose: self.log(ret_str)  # type: ignore

            if not failed:
                self._config_sent = True
                return True

        # There are various reasons for failure. One of the more common is due to swapping of cfg/data.
        if not autoretry_cfg_data:
            return False

        # we need to close config/data connections, and attempt to reconnect.
        # Swaps ports and their baud rates. Trying to reopen connections just does not work.
        self.log("Attempting to auto-resolve configuration error.")
        self._ser_config.reset_output_buffer()  # type: ignore
        self._ser_data.reset_output_buffer()  # type: ignore
        self._ser_config.reset_input_buffer()  # type: ignore
        self._ser_data.reset_input_buffer()  # type: ignore
        self._ser_config.baudrate = self._data_baud  # type: ignore
        self._ser_data.baudrate = self._config_baud  # type: ignore
        tmp = self._ser_data  # type: ignore
        self._ser_data = self._ser_config  # type: ignore
        self._ser_config = tmp

        self.log(
            f"Swapped opened config ({self._config_port_name}) and data ports ({self._data_port_name})."
        )
        self.log("Retrying configuration.")
        return self.send_config(config, max_retries, autoretry_cfg_data=False)

    async def start_sensor(self) -> None:
        """Starts the sensor and will place data into a queue.
        The goal of this function is to manage the state of the entire application. Nothing will happen if this function is not run with asyncio.
        This function attempts to read data from the sensor as quickly as it can, then extract data, and place data into an asyncio.Queue.
        Since this relies on the asyncio Queue, limitations may stem from asyncio. These issues, mainly revolving around thread safety, can be dealt with at the application layer.

        This function also actively attempts to context switch between intervals to minimize overhead.

        Raises:
            Exception: If sensor has some failure, will throw a SerialException.
        """
        self._last_t = time()

        if not self._is_alive:
            raise Exception("Disconnected sensor")

        if not self._config_sent:
            raise Exception("Config never sent to device")

        current_data: bytes = b""
        while True:
            await sleep(ASYNC_SLEEP)
            try:
                # Find our packet start
                current_data = self._ser_data.read_until(MAGIC_NUMBER)

                if current_data is None:
                    raise SerialException()

                new_data = self.parser.parse(self._ser_data)
                if new_data is None:
                    continue  # Packet was discarded. Try again.

                if self._active_data.full():
                    self._active_data.get_nowait()

                self._active_data.put_nowait(new_data)

            except (IndexError, ValueError) as _:
                pass

        return None

    async def get_data(self) -> Dict:
        """Returns data when it is ready. This function also updates the frequency measurement of the sensor.
        This function is blocking.

        Returns:
            dict: Sensor data
        """
        data = await self._active_data.get()
        tt = time()
        self._freq = (1 / (tt - self._last_t)) * 0.5 + self._freq * 0.5
        self._last_t = tt
        return data

    def get_data_nowait(self) -> Optional[Dict]:
        """Returns data if it is ready, otherwise none. This function also updates the frequency measurement of the sensor if data is available.

        Returns:
            Optional[dict]: Data if there is data available, otherwise returns None.
        """
        if self._active_data.full():
            tt = time()
            self._freq = (1 / (tt - self._last_t)) * 0.5 + self._freq * 0.5
            self._last_t = tt
            return self._active_data.get_nowait()

        return None

    def stop_sensor(self, send_stop: bool = True) -> None:
        """This function attempts to close all serial ports and update internal state booleans.

        Args:
            send_stop (bool, optional): Sends a sensorStop command to the conf port before disconnecting. Defaults to True.
        """

        # Send stop command
        if send_stop:
            try:
                self._ser_config.write(b"sensorStop\n")
                response = self._ser_config.read(100)
                if self._verbose:
                    self.log(f"sensorStop response: {response}")
            except:
                pass

        # Close ports
        try:
            self._ser_config.close()
            self._ser_data.close()
        except SerialException:
            pass

        self._update_alive()

    def get_update_freq(self) -> float:
        """Returns the frequency that the sensor is returning data at. This is not equivalent to the true capacity of the sensor, but rather the rate which the application is successfully getting data.

        Returns:
            float: Hz
        """
        return self._freq

    def __eq__(self, o: object) -> bool:
        return False

    def __repr__(self) -> str:
        return f"{self.model()} is alive: {self._is_alive} at {self._freq}Hz."
