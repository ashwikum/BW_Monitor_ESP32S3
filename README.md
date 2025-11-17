# BW_Monitor_ESP32S3

Firmware and display code for the **Waveshare ESP32-S3 Circular LCD 1.28"** display (non-touch version).

This project is based on the official [Arduino LVGL Demo](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.28/ESP32-S3-Touch-LCD-1.28-Demo.zip) from Waveshare for the [ESP32-S3-Touch-LCD-1.28](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.28).

## Hardware

- **Display:** Waveshare ESP32-S3 Circular LCD 1.28" (non-touch)
- **Microcontroller:** ESP32-S3
- Reference: [Waveshare ESP32-S3-Touch-LCD-1.28 Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.28)

## Overview

- Source files and fonts for a 1.28" circular LCD display using ESP32-S3
- Includes sensor and display drivers and example Arduino sketch `BW_Display.ino`
- Uses the same libraries included in the official Waveshare demo for both LVGL and SPI

## Setup & Configuration

### Board Version

Select **ESP32 Board version 2.0.12** in Arduino IDE or via `arduino-cli`.

### Arduino CLI Setup

Follow the configuration instructions from the [Waveshare ESP32-S3-Touch-LCD-1.28 Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.28).

## Build

Compile the project with verbose output:

```powershell
arduino-cli compile --verbose --fqbn esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,UploadSpeed=921600 .
```

## Upload

Upload to the connected ESP32-S3 (adjust `COM3` to your serial port):

```powershell
arduino-cli upload -p COM3 --verbose --fqbn esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,UploadSpeed=921600 .
```

## Notes

- The `--fqbn` parameters are configured for the non-touch Circular Display variant
- Use the libraries that ship with the Waveshare demo for compatibility
- Ensure you have the correct ESP32 board package (v2.0.12) installed
- Adjust the serial port (`-p COM3`) to match your system

## License

MIT
