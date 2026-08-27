# Eris

Eris is a real-time data acquisition (DAQ) and streaming framework for Arduino-based microcontrollers. It provides firmware running on an RTOS, with host-side drivers in Python, Matlab, and C (ROS2).

Designed for research applications in biosignal acquisition, biomechanics, and robotic control.

**Author:** Jonathan Camargo <jon-cama@gatech.edu>
**Firmware version:** v3.0

## Repository Structure

```
Eris/
  Firmware/
    Flavors/              All firmware variants (RTOS-agnostic)
      Eris/               Base firmware (reference implementation)
      ErisADS1299/        8-channel 24-bit EMG/ECG
      ErisADS131/         8-channel 24-bit low-power EMG
      ErisAnalog/         Configurable-channel analog EMG
      ErisBiom/           Biomechanics motion capture
      ErisBiom2/          Biomechanics v2 (with feature extraction)
      ErisDCMotor/        DC motor feedback and control
      ErisLeg/            Leg exoskeleton sensors
      ErisMPU/            6-DOF IMU
      ErisNextFlex*/      Flex sensor variants (4 flavors)
      ErisServoHand/      5-DOF robotic hand control
      ErisBici/           Bicycle interface (Arduino Nano)
      ErisTapok*/         CAN bus integration (2 flavors)
      BareMinimal/        Template for custom flavors
      ...                 See full list in Flavors/
  eriscommon/             Shared Arduino library (RTOS abstraction, buffers, protocol)
                          (lives under ../ArduinoLibraries/eriscommon — installed once
                          into your Arduino libraries folder)
  drivers/
    python/
      eris/               Python driver library (pip-installable, see drivers/python/README.md)
      Examples/           Usage examples and notebooks
      flavors/            Flavor-specific Python code
      ros_ws/             ROS workspace
      roshandlers/        ROS message handlers
    matlab/
      +eris/              Matlab bindings (SD card binary reader)
    c/
      serial/             Cross-platform serial library
      mypkg/              ROS2 package with COBS/SLIP encoding
  demos/                  End-to-end demos (e.g. servo_joystick)
  doc/                    Documentation and notes
                          - ADDING_A_SENSOR.md: how to add a sensor to a flavor
                          - COMMANDS.md: serial command reference
```

## Installation

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x), or PlatformIO
- A supported board — see the [Supported Targets](#supported-targets) matrix (Teensy, XIAO nRF52840, XIAO SAMD21, Due, Nano)

### Step 1: Install Board Support

**For Teensy (recommended):**

1. Install [Teensyduino](https://www.pjrc.com/teensy/td_download.html) add-on for Arduino IDE
2. In Arduino IDE: Tools > Board > Teensy 3.x or Teensy 4.x

**For Arduino Due:**

1. In Arduino IDE: Tools > Board > Boards Manager
2. Search "Arduino SAM Boards" and install it
3. Select Tools > Board > Arduino Due

### Step 2: Install RTOS Library (choose one)

All Eris flavors are RTOS-agnostic. Install **one** of the following:

**Option A: ChibiOS (recommended for Teensy)**

1. In Arduino IDE: Sketch > Include Library > Manage Libraries
2. Search for **"ChRt"** (ChibiOS/RT for Arduino)
3. Install it

**Option B: FreeRTOS**

1. In Arduino IDE: Sketch > Include Library > Manage Libraries
2. Search for **"FreeRTOS"** (Arduino_FreeRTOS by Richard Barry)
3. Install it

The firmware auto-detects which RTOS is installed at compile time via `eriscommon/eris_rtos.h`. No code changes needed.

### Step 3: Install Eris Libraries

Copy these directories from the repository into your Arduino libraries folder (typically `~/Arduino/libraries/` on Linux/Mac or `Documents\Arduino\libraries\` on Windows):

```
cp -r eriscommon/      ~/Arduino/libraries/eriscommon/
cp -r FastDualBuffer/  ~/Arduino/libraries/FastDualBuffer/
```

Or on Windows, copy the `eriscommon` and `FastDualBuffer` folders into `Documents\Arduino\libraries\`.

### Step 4: Install Third-Party Arduino Libraries

Install via Arduino Library Manager (Sketch > Include Library > Manage Libraries):

| Library | Required by | Notes |
|---------|-------------|-------|
| **SerialCommand** | All flavors | ASCII command parsing |
| **PacketSerial** | All flavors | COBS packet framing |
| **SD** | Flavors with SD card logging | Usually included with Teensyduino |
| **SPI** | SPI-based sensors | Usually included with Arduino core |
| **Wire** | I2C-based sensors | Usually included with Arduino core |

**Sensor-specific libraries** (install only for the flavor you are building):

| Library | Flavor(s) |
|---------|-----------|
| ADS1299 | ErisADS1299 |
| ADS131 | ErisADS131 |
| ADS1256 / ADS1256_4CH | ErisNextFlex variants |
| MPU9250 | ErisMPU, ErisBiom |
| PCA9685 | ErisServoHand |
| Filters | ErisADS1299, ErisBiom |
| FeatureExtractor | ErisADS1299, ErisBiom, ErisBiom2, ErisTapok2 |
| TimerOne | ErisNextFlex |
| FlexCAN | ErisTapok, ErisTapok2 |

### Step 5: Open and Upload a Flavor

1. Open the `.ino` file for your chosen flavor, e.g. `Firmware/Flavors/Eris/Eris.ino`
2. Select your board and port in Tools menu
3. Click Upload

**Quick test:** Start with `Firmware/Flavors/BareMinimal/` or `Firmware/Flavors/Eris/` -- these have the fewest external dependencies.

### Python Driver Installation

```bash
cd drivers/python/
pip install -e .
# or with optional analysis extras (pandas, h5py for SD-card binary parsing):
pip install -e ".[analysis]"
```

The driver is published as the `eris` package — see `drivers/python/README.md` for the API.

### ROS Integration (optional)

```bash
cd drivers/python/ros_ws/
catkin build
source devel/setup.bash
```

Requires ROS 1 (Melodic/Noetic) with `catkin_tools`.

## Supported Targets

Eris runs on any Arduino board with a ChibiOS or FreeRTOS port. **You never edit code
to switch RTOS** — `eriscommon/eris_rtos.h` picks it at compile time from the libraries
present (or, on boards that ship one, from the core).

| Board | MCU / core | RTOS (auto-selected) | BLE | Selected by | Build |
|-------|-----------|----------------------|-----|-------------|-------|
| **Teensy 3.x / 4.x** | Cortex-M4F | **ChibiOS** (ChRt) | — | `__has_include(<ChRt.h>)` | IDE + PlatformIO |
| **Seeed XIAO nRF52840** | Cortex-M4F | **FreeRTOS** (core-provided) | ✅ NUS | forced on nRF52 | Arduino IDE |
| **Seeed XIAO SAMD21** | Cortex-M0+ | **FreeRTOS** (Seeed lib) | — | FreeRTOS header present | IDE + PlatformIO |
| **Arduino Due** | Cortex-M3 | **ChibiOS** (ChRt) | — | `__has_include(<ChRt.h>)` | Arduino IDE |
| **Arduino Nano (AVR)** | ATmega328 | **ChibiOS** (ChRt) | — | `__has_include(<ChRt.h>)` | Arduino IDE |

Verified/primary targets: **Teensy** (ChibiOS), **XIAO nRF52840** (FreeRTOS + BLE),
**XIAO SAMD21** (FreeRTOS). Due and Nano work through the same abstraction but are
niche. Any other board with an `Arduino_FreeRTOS` / `STM32FreeRTOS` / `ChRt` port is
detected too.

### How the RTOS is chosen

`eris_rtos.h` resolves in this order (override by `#define ERIS_USE_CHIBIOS` or
`ERIS_USE_FREERTOS` before the include):

1. **nRF52 → FreeRTOS, always** — the core already links it; ChRt would clash on `SVC_Handler`.
2. else **`ChRt.h` present → ChibiOS**.
3. else an `*FreeRTOS.h` header present → **FreeRTOS**.

Stack tiers (`ERIS_STACK_*`) and heap/stack behavior differ per RTOS **and** CPU — read
`lessons_learned.md` §1 and §13 before tuning threads.

### Quick start

**Arduino IDE**
1. Install board support: Teensyduino · Seeed nRF52 Boards · Seeed SAMD Boards · Arduino SAM (Due).
2. Install the RTOS library for your target: **ChRt** (Teensy / Due / Nano) or **Seeed Arduino FreeRTOS** (XIAO SAMD21). **nRF52840 needs no RTOS lib** — FreeRTOS ships with the core.
3. Install the shared library: copy `../ArduinoLibraries/eriscommon` into your Arduino `libraries/` folder, plus **SerialCommand** and **PacketSerial** (Library Manager) and any sensor lib the flavor lists.
4. Open the flavor's `.ino` (e.g. `Firmware/Flavors/Eris/Eris.ino`), select board + port, **Upload**. (Detailed step-by-step is under [Installation](#installation) above.)
5. **BLE (nRF52 only):** uncomment `#define ERIS_USE_BLE` in the flavor's `configuration.h`; it advertises as `ERIS_BLE_NAME` and moves both data **and** commands onto Bluetooth. See `Firmware/NRF52_PORT_NOTES.md`.

**PlatformIO** (Teensy / SAMD21 envs)
```bash
pio run -e BareMinimal            # build
pio run -e ErisServo -t upload    # build + upload
pio device monitor -e ErisMPU     # serial monitor @ 115200
```
Wired envs: `Eris`, `BareMinimal`, `ErisServo`, `ErisADS1299`, `ErisMPU`, `ErisServoHand`. `eriscommon` resolves from `../ArduinoLibraries` automatically; other flavors build from the Arduino IDE.

**Host driver**
```bash
cd drivers/python && pip install -e .
```
Connect over USB serial, or — for nRF52 BLE builds — over Bluetooth (`drivers/python/eris/bleport.py`). See `drivers/python/README.md`.

## Flavors

Status legend: ✅ Verified working · 🟡 Updated, awaiting verification · 🟠 Experimental · ⚪ Reference / template

18 active flavors. RTOS is auto-selected per board (Teensy / Due / Nano → ChibiOS;
SAMD21 / nRF52840 → FreeRTOS — see [Supported Targets](#supported-targets)).

| Flavor | Status | Sensor / Purpose | Interface | Target board |
|--------|--------|------------------|-----------|--------------|
| `Eris` | 🟡 | Base reference (FSR + SineWave + Sync + optional SD) | Analog | Teensy |
| `BareMinimal` | ⚪ | Template — SineWave + serial/BLE only | — | Teensy / SAMD21 / nRF52 |
| `ErisServo` | ✅ | 6-channel servo, direct GPIO PWM (`Servo.h`) | PWM | Teensy / SAMD21 / Due |
| `ErisServoDriver` | 🟠 | 16-channel servo via PCA9685 (+ sine demo) | I2C | Teensy |
| `ErisServoHand` | 🟡 | 5-DOF robotic hand via PCA9685 | I2C | SAMD21 |
| `ErisServoINA` | 🟡 | PCA9685 servo + INA219 current monitor | I2C | Teensy |
| `ErisMPU` | ✅ | 6-DOF IMU (MPU9250) | I2C | SAMD21 |
| `ErisADS1299` | ✅ | 8-channel 24-bit EMG/ECG (+ SD) | SPI | Teensy |
| `ErisAnalog` | 🟡 | Configurable-channel analog EMG | ADC | Teensy |
| `ErisBiom2` | 🟠 | Biomechanics + on-device feature extraction | Mixed | Teensy |
| `ErisDCMotor` | 🟠 | DC motor feedback + control | PWM + ADC | Teensy |
| `ErisNextFlex` | 🟠 | EMG (ADS1256, 2-ch) + FSR + ETI | SPI | Teensy |
| `ErisNextFlexAnalog` | 🟠 | EMG (analog) + FSR + ETI | ADC | Teensy |
| `ErisNextFlexArray` | 🟠 | EMG array (ADS1256, 8-ch) + selector + SD | SPI | Teensy |
| `ErisNextFlexArrayAnalog` | 🟠 | EMG array (analog, 8-ch) + selector | ADC | Teensy |
| `ErisTapok2` | 🟠 | Wireless EMG over CAN + SD | CAN | Teensy |
| `ErisBici` | 🟠 | Bicycle interface (button + PWM) | Digital | Arduino Nano |
| `ErisBandwidthTest` | ⚪ | Serial throughput benchmark | — | Teensy |

**Archived** (moved to `Firmware/Flavors/old/`, superseded): `ErisADS131` → `ErisADS1299`, `ErisBiom` → `ErisBiom2`, `ErisTapok` → `ErisTapok2`, `ErisLeg` (dormant exo hardware).

See each flavor's `README.md` for pin assignments, sensor wiring, and the full per-flavor command set.

Most flavors also include FSR (force sensitive resistor) support on analog pins and a sync input for external trigger timestamping.

## Firmware Architecture

### RTOS Support

Eris uses real-time operating systems for deterministic sensor sampling. All flavors are **RTOS-agnostic** and compile against either ChibiOS or FreeRTOS with no code changes required.

- **ChibiOS** — primary on Teensy 3.x/4.x (also Due and AVR/Nano) via ChRt
- **FreeRTOS** — on SAMD21 (Seeed lib) and nRF52840 (core-provided, forced); the nRF52 build also adds optional BLE. See [Supported Targets](#supported-targets) for the full matrix.

#### RTOS Abstraction Layer

The abstraction is provided by `eriscommon/eris_rtos.h`, which auto-detects the installed RTOS at compile time:

```cpp
// Auto-detection (default): uses __has_include to find ChRt.h or FreeRTOS.h
#include <eris_rtos.h>

// Manual override (before including eris_rtos.h):
#define ERIS_USE_CHIBIOS    // Force ChibiOS
#define ERIS_USE_FREERTOS   // Force FreeRTOS
```

All firmware code uses portable `eris_*` macros and types:

| Abstraction | ChibiOS | FreeRTOS |
|-------------|---------|----------|
| `eris_thread_ref_t` | `thread_t*` | `TaskHandle_t` |
| `eris_binary_sem_t` | `binary_semaphore_t` | `SemaphoreHandle_t` |
| `eris_mutex_t` | `mutex_t` | `SemaphoreHandle_t` |
| `eris_mailbox_t` | `mailbox_t` | `QueueHandle_t` |
| `ERIS_CRITICAL_ENTER/EXIT()` | `chSysLockFromISR()` | `taskENTER_CRITICAL()` |
| `eris_thread_create(...)` | `chThdCreateStatic(...)` | `xTaskCreate(...)` |
| `eris_sleep_ms(ms)` | `chThdSleepMilliseconds(ms)` | `vTaskDelay(pdMS_TO_TICKS(ms))` |
| `eris_scheduler_start(fn)` | `chBegin(fn)` | `fn(); vTaskStartScheduler()` |

To switch RTOS, simply install the desired RTOS library in your Arduino environment — no firmware changes needed.

### Core Modules

Each firmware build is composed of these modules (namespaced):

| Module | Namespace | Purpose |
|--------|-----------|---------|
| serialcommand | `SerialCom` | Serial command parsing and dispatch |
| streaming | `Streaming` | Configurable feature streaming over serial |
| fsr | `FSR` | Force sensor reading (analog, 1kHz) |
| sinewave | `SineWave` | Test signal generator (1Hz sine) |
| sync | `Sync` | Digital input change detection with timestamps |
| sdcard | `SDCard` | Binary data logging to SD card |
| emg | `EMG` | EMG acquisition (ADS1299/ADS131/Analog, flavor-dependent) |

### Data Flow

```
Sensor ISR/Thread --> ErisBuffer<T> (circular buffer) --> Streaming thread --> COBS packet --> Serial TX
                                                      \-> SD card writer thread --> .bin file
```

- Each sensor runs in its own thread (or ISR for fast sensors like ADS1299)
- `ErisBuffer<T>` provides thread-safe circular buffering with missed-sample tracking
- The streaming thread reads buffers and packs data into COBS-encoded packets
- SD card logging runs independently on a separate thread

### Data Types

Defined in `customtypes.h`:

```cpp
EMGSample_t    { float timestamp; float ch[8]; }
FSRSample_t    { float timestamp; float ch[2]; }
IMUSample_t    { float timestamp; float ax,ay,az,wx,wy,wz; }
floatSample_t  { float timestamp; float value; }
uint8_tSample_t { float timestamp; uint8_t value; }
```

All timestamps are in milliseconds relative to `t0` (set at boot, resettable via `S_TIME`).

## Serial Protocol

### Encoding

Packets use **COBS** (Consistent Overhead Byte Stuffing) with `0x00` as the packet delimiter. This ensures no zero bytes appear in the payload, providing reliable framing.

### Packet Types

| Type byte | Meaning | Content |
|-----------|---------|---------|
| `'D'` | Data | Binary sensor samples |
| `'T'` | Text | ASCII info/debug messages |
| `'E'` | Error | Error messages |

### Commands

Send as ASCII text over serial, terminated with newline:

| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `S_F <feat1> <feat2> ...` | Set streaming features (e.g. `S_F SINE FSR`) |
| `S_ON` | Start streaming |
| `S_OFF` | Stop streaming |
| `S_TIME` | Synchronize time (reset t0) |
| `ON` / `OFF` | LED on/off |
| `START` / `KILL` | Start/stop sensor threads |
| `SD_REC <name>` | Start SD card recording with trial name |
| `SD_NREC` | Stop SD card recording |

### Baud Rate

- Console: 115200
- Python driver: 12,000,000 (12 Mbps over USB)

## Python Driver

### Usage

```python
from eris import Eris

# Connect and configure
e = Eris(features=['FSR', 'SineWave'],
         format=['FSR2CH', 'float'],
         port='COM3')

# Start streaming
e.start()

# Read data
while True:
    data = e.read()  # Returns dict with decoded samples
    # data['FSR'] -> list of FSRSample_t
    # data['SineWave'] -> list of floatSample_t

# Stop
e.stop()
```

### Examples

- `drivers/python/Examples/SineWave.py` -- Basic sinewave streaming
- `drivers/python/Examples/BandwidthTest.py` -- Serial throughput measurement
- `drivers/python/Examples/SineWaveROS.py` -- ROS-integrated streaming
- `drivers/python/Examples/BandwidthTest2.ipynb` -- Interactive bandwidth test
- `drivers/python/Examples/ROStest.ipynb` -- ROS topic latency measurement

## Matlab

### Reading SD Card Data

```matlab
data = eris.ReadBin('fsr/trial01.bin', 'Format', 'float');
% data.timestamp, data.fsr0, data.fsr1, ...
```

Reads binary files recorded by the SD card module. Parses the ASCII header row for column names and reads binary float data.

## C / ROS2 Integration

The `c/` directory contains a ROS2 package (`mypkg`) with:

- Cross-platform serial library
- `PacketSerial` template class for COBS/SLIP packet handling
- ROS2 node template for publishing Eris data to ROS topics

## Creating a New Flavor

1. Copy `Firmware/Flavors/BareMinimal/` to a new directory
2. Edit `configuration.h` for your pin assignments, buffer sizes, and sampling rates
3. Add sensor modules (`.h`/`.cpp`) following the namespace pattern
4. Register streaming functions in `streaming.cpp`
5. Add serial commands in `serialcommand.cpp`
6. Define data types in `customtypes.h`
7. Use `eris_*` macros from `<eris_rtos.h>` for all RTOS calls (threads, semaphores, critical sections)

All RTOS interactions should go through the abstraction layer — never call ChibiOS or FreeRTOS APIs directly. This ensures your flavor works with either RTOS.

## Configuration

Key settings in each flavor's `configuration.h`:

```cpp
#define FIRMWARE_VERSION "v3.0"
#define TXBUFFERSIZE 16       // Streaming buffer size
#define SDBUFFERSIZE 512      // SD card buffer size
#define FSR_NUMCHANNELS 2     // Number of FSR channels
#define FSR_FREQUENCY_HZ 1000 // FSR sampling rate
#define EMG_NUMCHANNELS 8     // Number of EMG channels
#define SDCARD true            // Enable/disable SD logging
```

## License

The MIT License (MIT) Copyright (c) 2020 Jonathan Camargo <jon-cama@gatech.edu>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Acknowledgments

Many thanks to Will Flanagan and James Lewis for their contributions to some of the functions in this repository.
