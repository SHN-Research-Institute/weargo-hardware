# WearGo Hardware

Arduino-based wearable device for motion tracking and sensor data collection using the BMI270/BMM150 IMU.

## Hardware

- Arduino Nano 33 BLE
- BMI270/BMM150 IMU (accelerometer, gyroscope, magnetometer)

## Features

- Real-time sensor data collection
- CSV output format for easy data processing
- 50ms sampling rate
- Serial output at 115200 baud

## Setup

1. Install PlatformIO IDE extension in VS Code
2. Open this project folder in VS Code
3. Connect your Arduino Nano 33 BLE
4. Build and upload the firmware

## Data Format

The device outputs sensor data in CSV format:

```
ax,ay,az,gx,gy,gz,mx,my,mz
```

Where:

- `ax,ay,az`: Accelerometer data (g)
- `gx,gy,gz`: Gyroscope data (°/s)
- `mx,my,mz`: Magnetometer data (µT)

## Dependencies

- Arduino_BMI270_BMM150 library (specified in platformio.ini)
