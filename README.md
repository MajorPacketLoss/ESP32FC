# ESP32FC – Fixed-Wing Flight Controller

ESP32-based flight controller for PWM receivers and four conventional servos. Core logic lives in [src/main.cpp](src/main.cpp).

## Features

- **Sensor fusion** with an MPU6050 IMU using dual Kalman filters via [`primeKalmanFilters`](src/main.cpp) and [`readIMU`](src/main.cpp).
- **Mode switchable stabilization**: raw pass-through below 1500 µs, PID-stabilized above 1500 µs; see [`loop`](src/main.cpp).
- **Independent PID loops** for roll, pitch, and yaw using [`rollPID`](src/main.cpp), [`pitchPID`](src/main.cpp), and [`yawPID`](src/main.cpp).
- **Failsafe RC handling**: signals time out after 50 ms in [`readChannel`](src/main.cpp).
- **Servo mirroring**: left/right aileron differential handled in [`applyServoOutputs`](src/main.cpp).

## Hardware

- ESP32 DevKit (3.3 V logic)
- MPU6050 IMU on I²C pins 22 (SDA) / 23 (SCL)
- PWM receiver inputs: CH1→GPIO27, CH2→GPIO25, CH4→GPIO14, CH6→GPIO26
- Servos: right aileron→GPIO21, left aileron→GPIO19, elevator→GPIO18, rudder→GPIO5

## Building

```bash
pio run
```

## Uploading & Monitor

```bash
pio run --target upload
pio device monitor -b 115200
```

## Calibration & Operation

1. Power on with the airframe still; [`configureMPU`](src/main.cpp) auto-calibrates.
2. Verify telemetry over serial (100 ms interval) from [`printTelemetry`](src/main.cpp).
3. Take off with stabilization off; engage stabilization mid-flight using CH6 ≥1500 µs.
4. Tune PID gains in [`rollPidSettings`](src/main.cpp), [`pitchPidSettings`](src/main.cpp), and [`yawPidSettings`](src/main.cpp) as needed.
