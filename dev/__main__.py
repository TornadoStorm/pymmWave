import asyncio

from config import SENSOR_CONF_PORT, SENSOR_DATA_PORT

from pymmWave.IWR6843AOP import IWR6843AOP
from pymmWave.utils import load_cfg_file


async def read_sensor(sensor: IWR6843AOP):
    while True:
        data = await sensor.get_data()
        print(data)


if __name__ == "__main__":
    sensor = IWR6843AOP("Main")
    config_file = load_cfg_file("dev/mmwave_configs/area_scanner_68xx_AOP.cfg")

    # CONFIG serial port
    config_connected = sensor.connect_config(SENSOR_CONF_PORT, 115200)
    if not config_connected:
        raise Exception(
            f"Failed to connect to mmWave sensor config port: {SENSOR_CONF_PORT}"
        )

    # DATA serial port
    data_connected = sensor.connect_data(SENSOR_DATA_PORT, 921600)
    if not data_connected:
        raise Exception(
            f"Failed to connect to mmWave sensor data port: {SENSOR_DATA_PORT}"
        )

    if not sensor.send_config(config_file, max_retries=1):
        raise Exception("Failed to send config file")

    event_loop = asyncio.new_event_loop()
    event_loop.create_task(sensor.start_sensor())
    event_loop.create_task(read_sensor(sensor))
    event_loop.run_forever()
