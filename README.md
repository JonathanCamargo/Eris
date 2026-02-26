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
  python/
    eris/                 Python driver library
    Examples/             Usage examples and notebooks
    flavors/              Flavor-specific Python code
    ros_ws/               ROS workspace
    roshandlers/          ROS message handlers
  matlab/
    +eris/                Matlab bindings (SD card binary reader)
  c/
    serial/               Cross-platform serial library
    mypkg/                ROS2 package with COBS/SLIP encoding
  doc/                    Documentation and notes
  dev/                    Test and debug scripts
```

## Installation

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x)
- A supported microcontroller board (Teensy 3.x/4.x or Arduino Due)

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
cd python/
pip install -r requirements.txt
```

This installs: `pandas`, `h5py`, `pyserial`, `construct`, `cobs`

### ROS Integration (optional)

```bash
cd python/ros_ws/
catkin build
source devel/setup.bash
```

Requires ROS 1 (Melodic/Noetic) with `catkin_tools`.

## Supported Hardware

**Microcontrollers:**
- Teensy 3.x / 4.x
- Arduino Due

**Sensors (by flavor):**

| Flavor | Sensor | Interface | Description |
|--------|--------|-----------|-------------|
| ErisADS1299 | ADS1299 | SPI | 8-channel 24-bit EMG/ECG bioamplifier |
| ErisADS131 | ADS131 | SPI | 8-channel 24-bit low-power EMG |
| ErisAnalog | Built-in ADC | Analog | Configurable-channel analog EMG |
| ErisMPU | MPU6050 | I2C | 6-DOF IMU (accelerometer + gyroscope) |
| ErisServoHand | PCA9685 | I2C | 5-DOF servo hand |
| ErisDCMotor | DC Motor | PWM | Motor feedback and control |
| ErisBiom / ErisBiom2 | Multi-sensor | Mixed | Biomechanics motion capture |
| ErisLeg | Pressure + IMU | Mixed | Leg exoskeleton sensors |
| ErisNextFlex* | Flex sensors | Various | 4 variants (analog, array, combined) |
| ErisBici | Button + PWM | Digital | Bicycle interface (Arduino Nano) |
| ErisTapok / ErisTapok2 | CAN bus | CAN | Tapok sensor integration |
| Potentiometer | Potentiometer | Analog | Position sensing |
| BareMinimal | None | -- | Template for custom flavors |
| ErisBandwidthTest | None | -- | Serial throughput testing |

Most flavors also include FSR (force sensitive resistor) support on analog pins and a sync input for external trigger timestamping.

## Firmware Architecture

### RTOS Support

Eris uses real-time operating systems for deterministic sensor sampling. All flavors are **RTOS-agnostic** and compile against either ChibiOS or FreeRTOS with no code changes required.

- **ChibiOS** -- Mature, well-tested on Teensy 3.x/4.x
- **FreeRTOS** -- Lightweight alternative, works on Teensy and Arduino Due

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

- `python/Examples/SineWave.py` -- Basic sinewave streaming
- `python/Examples/BandwidthTest.py` -- Serial throughput measurement
- `python/Examples/SineWaveROS.py` -- ROS-integrated streaming
- `python/Examples/BandwidthTest2.ipynb` -- Interactive bandwidth test
- `python/Examples/ROStest.ipynb` -- ROS topic latency measurement

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
