# Firmware

Firmware for the [CreaTeBME](https://github.com/CreaTe-M8-BME/CreaTeBME) sensors.

No additional libraries are required. You only need to make sure you have the 2.0.0 version of the esp32 board manager installed. And are using the `WEMOS LOSIN32` board.

## Flashing firmware

If you want to flash multiple IMUs quickly without recompiling the firmware every time in the Arduino IDE, you can use the Python scripts provided as follows:

### 1. Setup
Run the following command in the terminal to install esptool and setuptools from the pipfile:
```bash
pipenv install
```

> [!NOTE]
>  1. Make sure you have `pipenv` installed. If not then you can install it with:
>        ```bash
>        python -m pip install pipenv --user
>        ```
>  2. If you encounter an error with `module 'pkgutil` then run the following command to fix your pip:
>     ```bash
>     python -m ensurepip --upgrade
>     ```
> 
>  2. If you are using a different python version than 3.12 and you get an error locking the pipenv file then you can run:
>     ```bash
>     pipenv install esptool setuptools --python [YOUR VERSION (e.g. 3.12)]`.
>     ```


### 2. Compiling the code
After every code change you have to recompile the arduino code. You do not have to do this if the code hasn't changed.<br>

If you haven't already, install the board manager for the ESP32 by Espressif. make sure you install version `2.0.0`.
And select the `WEMOS LOLIN32` board in the Arduino IDE.

In your Arduino IDE, export your sketch as a series of .bin files with `Sketch` -> `Export Compiled Binary`.

> [!WARNING]
> Make sure you use version `2.0.0` of the `esp32` boards manager by Espressif. Earlier versions do not compile the firmware correctly and newer versions have a bug with the `wire` library that prevents the IMU from working correctly. [Installation guide.](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)

### 3. Flashing the firmware
Repeat this step for every sensor you want to flash. It will automatically detect the **first** connected sensor.

1. Connect your IMU with a USB cable.
2. Run `python flash_firmware.py` to flash your sensor.
   1. Alternatively, you can run it directly in the terminal with:
        ```bash
        pipenv run esptool -b 460800 --before default_reset --after hard_reset --chip esp32 write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 ./esp32_bluetooth_host/build/esp32.esp32.lolin32/esp32_bluetooth_host.ino.bootloader.bin 0x8000 ./esp32_bluetooth_host/build/esp32.esp32.lolin32/./esp32_bluetooth_host.ino.partitions.bin 0x10000 ./esp32_bluetooth_host/build/esp32.esp32.lolin32/./esp32_bluetooth_host.ino.bin
        ```

