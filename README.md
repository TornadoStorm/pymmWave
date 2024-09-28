# pymmWave

This is a python package built upon the original [pymmWave](https://github.com/pfeghali/pymmWave) package by pfeghali for the TCS Design Project 2024/2025 Group 6 project. This package extends the functionality and modularity for reading and transforming sensor data from Texas Instruments mmWave sensors with different firmware and configurations.

## Installation

Run the following code in your terminal to get the latest version:

```console
$ pip install git+ssh://git@gitlab.utwente.nl/dp-6/pymmwave.git@main#egg=pymmwave
```

## Quickstart

Ensure you have flashed the firmware for the Industrial Toolbox's [Area Scanner](https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_12_1/labs/Area_Scanner/docs/area_scanner_users_guide.html) project.

The following example code reads and prints the parsed and transformed sensor data. Make sure the serial ports are correctly configured to point to the sensor's DATA and CONF ports, and that the sensor's height and elevation tilt match the settings below.

```python
from pymmWave.parsing.area_scanner.area_scanner_parser import AreaScannerParser
from pymmWave.utils import load_cfg_file
from pymmWave.sensor import Sensor
from pymmWave.IWR6843AOP import IWR6843AOP
import asyncio
import json

sensor1 = IWR6843AOP("1", verbose=False)
file = load_cfg_file("./example_configs/are_scanner_AOP.cfg")

parser = AreaScannerParser()
parser.height = 1.3
parser.elevation_tilt = np.radians(-15)
sensor1.parser = parser

# Your CONFIG serial port name
config_connected = sensor1.connect_config('/dev/tty.SLAB_USBtoUART4', 115200)
if not config_connected:
    print("Config connection failed.")
    exit()

# Your DATA serial port name
data_connected = sensor1.connect_data('/dev/tty.SLAB_USBtoUART', 921600)
if not data_connected:
    print("Data connection failed.")
    exit()

if not sensor1.send_config(file, max_retries=1):
    print("Sending configuration failed")
    exit()

# Basic class to print data from the sensor
async def print_data(sens: Sensor):
    await asyncio.sleep(2)
    while True:
        sensor_data = await sens.get_data()
        print(json.dumps(sensor_data))

event_loop = asyncio.new_event_loop()

event_loop.create_task(sensor1.start_sensor())
event_loop.create_task(print_data(sensor1))
event_loop.run_forever()
```
