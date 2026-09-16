# Eris — What It Is and How It Fits Together

Eris is a real-time data acquisition and streaming framework for Arduino-class
microcontrollers. You give it a sensor and a sampling rate; it gives you
timestamped samples arriving on a host computer, deterministically, without you
writing a scheduler, a ring buffer, a framing protocol, or a decoder.

It was built for research work — biosignals, biomechanics, robotic control —
where "roughly 100 Hz, mostly" is not good enough, and where the person wiring up
the sensor is usually not the person who wants to write firmware.

- **Author:** Jonathan Camargo (Georgia Tech) · **License:** MIT · **Firmware:** v3.0
- **Repository:** <https://github.com/JonathanCamargo/Eris>
- **Shared Arduino library (`eriscommon`, separate repo):**
  <https://github.com/Robiolab/ArduinoLibraries>
- **Repo layout, install steps, per-board matrix:** [README.md](README.md)
- **Command reference:** [doc/COMMANDS.md](doc/COMMANDS.md)
- **Adding your own sensor:** [doc/ADDING_A_SENSOR.md](doc/ADDING_A_SENSOR.md)

---

## The problem Eris solves

The naive Arduino data logger is a `loop()` that reads a sensor and calls
`Serial.print`. It works until any of these show up, and in research they all do:

- **Two sensors at different rates.** A 1 kHz analog channel and a 100 Hz IMU
  cannot share one `loop()` without one of them jittering.
- **Timestamps you can trust.** `millis()` inside a loop that also does I/O
  records when the loop got around to it, not when the sample happened.
- **Binary data over a serial link.** Sensor values contain zero bytes; newlines
  are not a safe frame delimiter; a dropped byte desynchronizes you forever.
- **A host that has to know the layout.** Hand-written struct definitions on both
  ends drift apart the first time someone adds a channel.

Eris answers each of those with one mechanism, and those four mechanisms are
essentially the whole framework.

| Problem | Eris mechanism |
|---|---|
| Concurrent sampling at different rates | An RTOS thread per sensor, at a fixed period |
| Honest timestamps | Sample stamped in the sampling thread, from a resettable `t0` |
| Safe binary framing | COBS packets with a `0x00` delimiter |
| Host/firmware layout drift | The firmware describes its own wire format (`DESC`) |

---

## The mental model

```
   ┌──────────────┐   samples    ┌──────────────┐   packets   ┌───────────────┐
   │ sensor thread│ ───────────► │  ErisBuffer  │ ──────────► │   Streaming   │
   │  (fixed Hz)  │              │  (circular)  │             │    thread     │
   └──────────────┘              └──────────────┘             └───────┬───────┘
   ┌──────────────┐              ┌──────────────┐                     │
   │ sensor thread│ ───────────► │  ErisBuffer  │ ──────────►         │
   └──────────────┘              └──────┬───────┘                     ▼
                                        │                     COBS → Serial / BLE
                                        ▼                             │
                                 SD writer thread                     ▼
                                  (.bin on card)               Python / Matlab / ROS
```

Four ideas:

1. **A sensor is a thread.** It wakes on a fixed period, reads the device,
   stamps the sample, and appends it to its own buffer. It never prints, never
   waits on the host, and never shares a bus with another thread.
2. **A buffer decouples producer from consumer.** `ErisBuffer<T>` is a
   thread-safe circular buffer that also tracks *missed* samples — so an
   overrun is visible in the data rather than silently swallowed.
3. **Streaming is a separate, selectable concern.** One thread drains whichever
   buffers you asked for and frames them. What gets streamed is chosen at
   runtime by a command, not at compile time.
4. **The transport is interchangeable.** USB serial, Bluetooth (nRF52 NUS), or
   an SD card — same samples, same packets.

Everything else in the repo is either an implementation of one of those, or a
particular set of sensors wired into them.

---

## Flavors: one firmware per rig, not one firmware with switches

Eris does not ship a single configurable binary. It ships **flavors** — small,
self-contained sketch directories, one per hardware setup, under
`Firmware/Flavors/`. There are 18 active ones plus an `old/` archive.

A flavor is:

```
Firmware/Flavors/ErisMPU/
  ErisMPU.ino          setup(), pin config, ERIS_RUN(start)
  configuration.h      pins, rates, channel counts, buffer sizes
  customtypes.h        the sample structs + their descriptors
  imu.cpp / imu.h      the sensor itself
  streaming.cpp        which feature names map to which buffers
  serialcommand.cpp    the flavor's extra ASCII commands
  README.md            wiring, pins, command set
```

This is deliberate. Arduino compiles a sketch directory, so shared code is
duplicated rather than `#include`d across flavors — the cost is drift, which
`tools/check_drift.py` exists to detect. The benefit is that a student can copy
one directory, change three files, and have working firmware without
understanding the other seventeen.

**Where to start:** `BareMinimal` (template — sine wave and serial only),
`Eris` (base reference), or `ErisMPU` / `ErisServo` / `ErisADS1299` (verified,
real sensors). The README's flavor table marks each one ✅ verified, 🟡 awaiting
verification, 🟠 experimental, or ⚪ reference.

---

## RTOS-agnostic by construction

Eris runs on ChibiOS *or* FreeRTOS, and **flavor code never names either one**.
All RTOS calls go through `eris_*` macros in
`ArduinoLibraries/eriscommon/src/eris_rtos.h`, which picks the RTOS at compile
time from the board and the libraries present:

| Board | RTOS chosen | Why |
|---|---|---|
| Teensy 3.x / 4.x | ChibiOS (ChRt) | Primary target |
| XIAO nRF52840 | FreeRTOS (core-provided) | Forced — ChRt would clash on `SVC_Handler` |
| XIAO SAMD21 | FreeRTOS (Seeed lib) | Header present |
| ESP32 | FreeRTOS (ESP-IDF) | Scheduler already running; tasks pinned to a core |
| Due / Nano | ChibiOS (ChRt) | Same abstraction, niche targets |

Override with `#define ERIS_USE_CHIBIOS` or `#define ERIS_USE_FREERTOS` before
the include. The abstraction covers threads, critical sections, binary
semaphores, mutexes, mailboxes, sleeps, and scheduler start:

```cpp
ERIS_RUN(start);            // start the scheduler from setup(), per-RTOS
eris_sleep_ms(10);          // portable thread delay
ERIS_CRITICAL_ENTER();      // ISR-safe critical section
```

**Rule for contributors:** never call `chThdSleepMilliseconds` or `vTaskDelay`
directly in a flavor. If the abstraction is missing something, add it to
`eris_rtos.h` — that keeps every flavor portable rather than one flavor pinned.

Note that `eriscommon` lives *outside* this repo, in the
[Robiolab/ArduinoLibraries](https://github.com/Robiolab/ArduinoLibraries)
repository, and must be installed into your Arduino `libraries/` folder before
anything compiles. Clone it as a sibling of the Eris checkout — the PlatformIO
build resolves it at `../ArduinoLibraries`:

```bash
git clone https://github.com/JonathanCamargo/Eris.git
git clone https://github.com/Robiolab/ArduinoLibraries.git
```

---

## Declaring a sensor

Most sensors are a periodic poll, and for those the boilerplate is generated.
Two blocks — a described struct and a sensor declaration — produce a working
thread:

```c
// customtypes.h — the struct and its descriptor, kept adjacent on purpose
typedef struct {
  float timestamp;
  float fx, fy, fz;
} ForceSample_t;

ERIS_DESCRIBE_BEGIN(ForceSample_t)
  ERIS_TIME (ForceSample_t, timestamp)
  ERIS_FLOAT(ForceSample_t, fx)
  ERIS_FLOAT(ForceSample_t, fy)
  ERIS_FLOAT(ForceSample_t, fz)
ERIS_DESCRIBE_END(ForceSample_t)
```

```c
// force.cpp — you write the device reads; the macro writes the rest
ERIS_SENSOR(Force, ForceSample_t, 100) {
  s.fx = analogRead(ERIS_ADC(0)) * FORCE_SCALE;
  s.fy = analogRead(ERIS_ADC(1)) * FORCE_SCALE;
  s.fz = analogRead(ERIS_ADC(2)) * FORCE_SCALE;
  return true;              // false = skip this tick, append nothing
}
```

`ERIS_SENSOR` generates `Force::buffer`, `Force::start()`, the working area, the
stack tier, the priority, a drift-free 100 Hz loop, and the timestamp. For N
devices on one bus, `ERIS_SENSOR_MULTI(Force, ForceSample_t, 100, 2)` polls them
in sequence from a *single* thread — devices sharing an I²C bus must have one
owner, or their transactions corrupt each other.

For the very common `float timestamp; float ch[N];` shape there is a one-liner,
`ERIS_DESCRIBE_CHANNELS(ForceSample_t, ch, FORCE_NUMCHANNELS)`, which reads the
same constant the struct does — so raising the channel count updates the
descriptor automatically.

Two things are checked at build time rather than left to be discovered in data:

- `ERIS_DESCRIBE_END` asserts the described fields cover **every byte** of the
  struct. Forget a field, or gain silent compiler padding, and the build stops.
- The rate must divide 1000 evenly (250, 200, 125, 100, 50, 20, 10 — not 300 or
  60), so the period lands on a whole scheduler tick.

Interrupt-driven, DMA, and multi-rate sensors are *not* a fit for the macro and
stay hand-written; `ErisADS1299` is the reference for that shape.

Full walkthrough, including streaming registration: [doc/ADDING_A_SENSOR.md](doc/ADDING_A_SENSOR.md).

---

## The wire protocol

Samples leave the device as **COBS**-encoded packets delimited by `0x00`. COBS
guarantees no zero byte appears inside the payload, so the delimiter is
unambiguous and a receiver can always resynchronize after a dropped byte. Each
packet carries a type byte:

| Type | Meaning |
|---|---|
| `'D'` | Binary sensor samples |
| `'T'` | Text — info and debug |
| `'E'` | Error |

Control goes the other way as plain newline-terminated ASCII, so you can drive a
device from a serial terminal with no host library at all:

```
INFO                 firmware info
S_F IMU_0 SINE       select which features to stream
S_ON / S_OFF         start / stop streaming
S_TIME               reset t0 so timestamps are zero-based
DESC                 make the firmware describe its own wire format
S_MODE ASCII         human-readable output instead of binary
SD_REC trial01       start SD recording
```

Console baud is 115200; the Python driver runs the link at 12 Mbps over USB CDC.

### Two output modes

**Binary** is the real one — every sample, COBS-framed, decoded by the host driver.

**ASCII** (`S_MODE ASCII`) prints `IMU_0.ax:0.123,IMU_0.ay:-0.045,...`, which the
Arduino Serial Plotter auto-labels and graphs with zero host code. It is
deliberately a **monitor, not a log**: newest sample only, 20 Hz, timestamps
excluded by default because a monotonically rising trace destroys the Plotter's
autoscale. Use it to confirm a sensor is alive; use binary for data you keep.

### `DESC` — the firmware is the schema

The failure mode this kills: someone adds a channel to the firmware, the host
struct still says six floats, and every subsequent recording is quietly
misaligned.

`DESC` makes the device report the byte layout of the features currently
selected:

```
DESC IMU_0 timestamp:f32:t,ax:f32,ay:f32,az:f32,wx:f32,wy:f32,wz:f32
DESC END
```

Fields are `name:type[:role]` — types `f32 u8 i8 u16 i16 u32 i32 pad`, roles
`:t` time, `:m` meta, `:p` padding. Crucially, the description is emitted
through the *same* `StreamSamples()` calls in the *same* order as the data, so
the schema cannot drift from the bytes without the data drifting too.

Which is why the host no longer needs to be told the format:

```python
from eris import Eris

e = Eris(['IMU_0', 'IMU_1'], port='COM3')   # layout fetched via DESC
e.start()
while True:
    data = e.read()          # {'IMU_0': [...], 'IMU_1': [...]}
e.stop()
```

A sample type with no `ERIS_DESCRIBE` block answers `UNDESCRIBED`, and the host
falls back to an explicit `format=` — which is exactly the tax that motivates
describing your types.

---

## Host side

| Host | Location | Use |
|---|---|---|
| **Python** | `drivers/python/` (`pip install -e .`) | Primary driver; live streaming, `DESC` auto-config, examples and notebooks |
| **BLE** | `drivers/python/eris/bleport.py` | pyserial-compatible port over `bleak`, for nRF52 builds |
| **Matlab** | `drivers/matlab/+eris/` | Reads SD-card `.bin` recordings |
| **C / ROS2** | `drivers/c/` | Serial library + ROS2 node template |
| **ROS 1** | `drivers/python/ros_ws/` | catkin workspace, topic publishing |

BLE is verified at a full 100 Hz on nRF52840, but only with the right link
settings — a 247-byte MTU, a short connection interval, and a backpressure-aware
`write()`. If COBS frames start failing wholesale over Bluetooth, it is byte
loss on the link, not a framing bug. `lessons_learned.md` §12 has the detail.

---

## Getting to first data

1. Install board support and the RTOS library for your target (Teensy → **ChRt**;
   XIAO SAMD21 → **Seeed Arduino FreeRTOS**; nRF52840 → nothing, the core ships
   FreeRTOS).
2. Clone [Robiolab/ArduinoLibraries](https://github.com/Robiolab/ArduinoLibraries)
   next to your Eris checkout and copy `ArduinoLibraries/eriscommon` into your
   Arduino `libraries/` folder, plus **SerialCommand** and **PacketSerial**.
3. Open `Firmware/Flavors/BareMinimal/BareMinimal.ino`, select board and port,
   upload.
4. Open Serial Monitor at 115200 and send `S_F SINE`, then `S_MODE ASCII`, then
   `S_ON`. Switch to Serial Plotter to watch the sine.
5. `pip install -e drivers/python`, then run `drivers/python/Examples/SineWave.py`
   for the binary path.

With PlatformIO, six flavors are wired as envs: `pio run -e BareMinimal`,
`pio run -e ErisServo -t upload`, `pio device monitor -e ErisMPU`.

Step-by-step installation, the third-party library table, and the per-flavor
dependency list are in the [README](README.md#installation).

---

## Conventions worth internalizing

- **Every sample starts with `float timestamp`**, in milliseconds from `t0`,
  stamped by the sampling thread. `S_TIME` resets `t0`.
- **One namespace per module**: `FSR::`, `EMG::`, `SineWave::`, `Streaming::`,
  `SerialCom::`, `SDCard::`, `Sync::`.
- **One thread per sensor, one owner per bus.**
- **`ERIS_ADC(n)`, never `A1`/`A2`.** ESP32 has no `A1`; code using the aliases
  compiles on Teensy and then breaks or reads the wrong pin elsewhere.
- **`ERIS_NORMAL_PRIORITY`, not raw `NORMALPRIO`.**
- **Shared code is duplicated across flavors on purpose** — run
  `python tools/check_drift.py` before you trust that two flavors agree.

---

## Where to look next

| Question | File |
|---|---|
| How do I install and build? | [README.md](README.md) |
| What commands does the firmware take? | [doc/COMMANDS.md](doc/COMMANDS.md) |
| How do I add my own sensor? | [doc/ADDING_A_SENSOR.md](doc/ADDING_A_SENSOR.md) |
| Why does the RTOS / BLE / ESP32 behave this way? | `lessons_learned.md` |
| What does this flavor's wiring look like? | `Firmware/Flavors/<Name>/README.md` |
| What can I change per flavor? | `Firmware/Flavors/<Name>/configuration.h` |
| Where is the source? | [JonathanCamargo/Eris](https://github.com/JonathanCamargo/Eris) |
| Where is `eriscommon`? | [Robiolab/ArduinoLibraries](https://github.com/Robiolab/ArduinoLibraries) |

To find out what a flavor really supports, the source is authoritative:

```bash
grep addCommand Firmware/Flavors/<Flavor>/serialcommand.cpp
grep strncmp    Firmware/Flavors/<Flavor>/streaming.cpp
```
