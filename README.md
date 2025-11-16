# BW_Monitor_ESP32S3

Firmware and display code for an ESP32-S3-based monitor/display.

Overview
- Source files and fonts for a 1.28" LCD display using ESP32-S3.
- Includes sensor and display drivers and example Arduino sketch `BW_Display.ino`.

Build
This project uses the Arduino CLI. To build locally run:

```powershell
arduino-cli compile --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,UploadSpeed=921600" .
```

Upload
Set your serial port and upload with:

```powershell
arduino-cli upload -p COM3 --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,UploadSpeed=921600" .
```

Notes
- Adjust the `--fqbn` and port to match your board and environment.
- If you prefer PlatformIO or Arduino IDE, adapt accordingly.

License
MIT (add LICENSE file if you want a different license)
