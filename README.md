# Firmware

Firmware for the [CreaTeBME](https://github.com/CreaTe-M8-BME/CreaTeBME) sensors.

No additional libraries are required. You only need to make sure you have the latest version of the esp32 board manager installed.

## Flashing firmware without needing to compile each time

If you want to flash multiple IMUs quickly without recompiling the firmware every time in the Arduino IDE, you can use the Python scripts provided as follows:

1. Run the setup script to install esptool and setuptools with `python setup_esptool.py`.
2. In your Arduino IDE, export your sketch as a series of .bin files with Sketch -> Export Compiled Library (you only need to do this once every time your code is updated).
3. Connect your IMU with a USB cable.
4. Run `python flash_firmware.py` to flash your sensor.

> [!NOTE]
> If you get an error regarding the bootloader, make sure to update your esp32 boards manager and recompile the .bin file