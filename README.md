# pymmWave

This is a python package built upon the original [pymmWave](https://github.com/pfeghali/pymmWave) package by pfeghali for the TCS Design Project 2024/2025 Group 6 project. This package extends the functionality and modularity for reading and transforming sensor data from Texas Instruments mmWave sensors with different firmware and configurations.

## Quickstart

Ensure you have flashed the firmware for the Industrial Toolbox's [Area Scanner](https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_12_1/labs/Area_Scanner/docs/area_scanner_users_guide.html) project, and installed all required dependencies in this project's **requirements.txt** file. You can then run **dev/\_\_main.py\_\_** and have a look at its contents for a basic demo of how to use this package to read sensor data. Take note of the constants defined in **dev/config.py** to configure your sensor's values.

## Custom / Different Sensor Firmware

If the output data of your sensor differs from the default Area Scanner output data, you may implement your own parser for the sensor. See the implementation of the AreaScannerParser class for how to make your own parser.
