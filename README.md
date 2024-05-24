# Firmware

Firmware for the [CreaTeBME](https://github.com/CreaTe-M8-BME/CreaTeBME) sensors.

No additional libraries are required. You only need to make sure you have the latest version of the esp32 board manager installed.

## Flashing firmware

If you want to flash multiple IMUs quickly without recompiling the firmware every time in the Arduino IDE, you can use the Python scripts provided as follows:

1. Run the setup script to install esptool and setuptools with `python setup_esptool.py`.
   1. Alternatively, you can install the required packages manually with:
        ```bash
        pipenv install
        ```

> [!NOTE]
>  1. Make sure you have `pipenv` installed. If not then you can install it with:
>        ```bash
>        pip install pipenv --user
>        ```
>  2. If you are using an older version and you get an error locking the pipenv file then you can run
>        ```bash
>        pipenv install esptool setuptools --python [YOUR VERSION (e.g. 3.12)]`.
>        ```
2. In your Arduino IDE, export your sketch as a series of .bin files with `Sketch` -> `Export Compiled Binary` (you only need to do this once every time your code is updated).
3. Connect your IMU with a USB cable.
4. Run `python flash_firmware.py` to flash your sensor.
   1. Alternatively, you can run it directly in the terminal with:
        ```bash
        pipenv run esptool -b 460800 --before default_reset --after hard_reset --chip esp32 write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 ./esp32_bluetooth_host/build/esp32.esp32.lolin32/esp32_bluetooth_host.ino.bootloader.bin 0x8000 ./esp32_bluetooth_host/build/esp32.esp32.lolin32/./esp32_bluetooth_host.ino.partitions.bin 0x10000 ./esp32_bluetooth_host/build/esp32.esp32.lolin32/./esp32_bluetooth_host.ino.bin
        ```

> [!WARNING]
> Make sure you use version `2.0.0` of the `esp32` boards manager by Espressif. Earlier versions do not compile the firmware correctly and newer versions have a bug with the `wire` library that prevents the IMU from working correctly. [Installation guide.](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
