# Firmware

Firmware for the [CreaTeBME](https://github.com/CreaTe-M8-BME/CreaTeBME) sensors.

No additional libraries are required. You only need to make sure you have the latest version of the esp32 board manager installed.

## Flashing firmware

If you want to flash multiple IMUs quickly without recompiling the firmware every time in the Arduino IDE, you can use the Python scripts provided as follows:

1. Run the setup script to install esptool and setuptools with `python setup_esptool.py`.
2. In your Arduino IDE, export your sketch as a series of .bin files with `Sketch` -> `Export Compiled Binary` (you only need to do this once every time your code is updated).
3. Connect your IMU with a USB cable.
4. Run `python flash_firmware.py` to flash your sensor.

> [!WARNING]
> Make sure you use version `2.0.0` of the `esp32` boards manager by Espressif. Earlier versions do not compile the firmware correctly and newer versions have a bug with the `wire` library that prevents the IMU from working correctly. [Installation guide.](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
