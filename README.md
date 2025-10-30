# ESP32FC – Fixed-Wing Flight Controller

ESP32-based flight controller for PWM receivers and four conventional servos. Core logic lives in [src/main.cpp](src/main.cpp).

## Features

- **Sensor fusion** with an MPU6050 IMU using dual Kalman filters via [`primeKalmanFilters`](src/main.cpp) and [`readIMU`](src/main.cpp).
- **Mode switchable stabilization**: raw pass-through below 1500 µs, PID-stabilized above 1500 µs; see [`loop`](src/main.cpp).
- **Independent PID loops** for roll, pitch, and yaw using [`rollPID`](src/main.cpp), [`pitchPID`](src/main.cpp), and [`yawPID`](src/main.cpp).
- **Failsafe RC handling**: signals time out after 50 ms in [`readChannel`](src/main.cpp).
- **Selectable aileron mirroring**: optional left/right inversion via `INVERT_LEFT_AILERON` in [`loop`](src/main.cpp).

## Hardware

- ESP32 DevKit (3.3 V logic)
- MPU6050 IMU on I²C pins 22 (SDA) / 23 (SCL)
- PWM receiver inputs: CH1→GPIO27, CH2→GPIO25, CH4→GPIO14, CH6→GPIO26
- Servos: right aileron→GPIO21, left aileron→GPIO19, elevator→GPIO18, rudder→GPIO5

### PCB port mapping

- Servo headers: `S4`→right aileron, `S3`→left aileron, `S2`→elevator, `S1`→rudder. If the ailerons move opposite to stick input, swap the plugs in `S4` and `S3` instead of rewiring the linkage.
- Receiver headers: `C1`→channel 1 (aileron), `C2`→channel 2 (elevator), `C3`→channel 4 (rudder), `C4`→channel 6 or 5 (stabilization mode switch).

## Rebuilding the Project

### Option A – PlatformIO (recommended)

1. Install VS Code and the PlatformIO extension.
2. Clone this repository: `git clone <repo-url> && cd ESP32FC`.
3. Build: `pio run`.
4. Upload: `pio run --target upload`.
5. Monitor telemetry: `pio device monitor -b 115200`.

PlatformIO automatically fetches library dependencies pinned in `platformio.ini`.

#### Building

```bash
pio run
```

Compiles the PlatformIO project using the environment and libraries defined in `platformio.ini`.

#### Uploading & Monitor

```bash
pio run --target upload
```

Builds and flashes the firmware to the connected ESP32.

```bash
pio device monitor -b 115200
```

Opens the serial monitor at 115200 baud to view telemetry output.

### Option B – Arduino IDE

1. Install the ESP32 board support package via Boards Manager.
2. Download the zipped dependency bundle from [`lib/dependencies.zip`](lib/dependencies.zip) and install each ZIP through **Sketch → Include Library → Add .ZIP Library**.
3. Copy the contents of `src/main.cpp` into an Arduino sketch folder named `ESP32FC`.
4. Select **Tools → Board → ESP32 Dev Module**, choose the correct COM port, and upload.

## Calibration & Operation

1. Power on with the airframe still; [`configureMPU`](src/main.cpp) auto-calibrates.
2. Verify telemetry over serial (100 ms interval) from [`printTelemetry`](src/main.cpp).
3. Take off with stabilization off; engage stabilization mid-flight using CH6 ≥1500 µs.
4. Tune PID gains in [`rollPidSettings`](src/main.cpp), [`pitchPidSettings`](src/main.cpp), and [`yawPidSettings`](src/main.cpp) as needed.

## Acknowledgements

Generative AI (GitHub Copilot) assisted with code and documentation throughout this project.
