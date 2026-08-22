# ErisESP32

Minimal Eris flavor for the **ESP32 family**, modelled on `BareMinimal`.
Streams a synthetic sine wave over serial and answers the universal command set
(`INFO`, `ON`/`OFF`, `SINE`, `S_F`, `S_TIME`, `START`, `KILL`, ...).

Use it as the starting point for ESP32 work: add your sensor module the same way
the other flavors do (namespace + thread + `ErisBuffer`), then register a
streaming function in `streaming.cpp`.

## Why a separate flavor

The existing flavors carry Teensy pin maps (`PIN_LED 13`, `PIN_LED_R 28`,
`PIN_SYNC 29`) and, in `BareMinimal`'s case, the nRF52 BLE transport. Those pins
do not exist on ESP32, so this flavor exists to hold an ESP32-correct
`configuration.h` rather than bend one of the verified boards out of shape.

`streaming.cpp`, `streaming.h`, `customtypes.h` and `serialcommands.h` are byte
copies of `BareMinimal`'s, so `tools/check_drift.py` stays quiet.

## What is different on ESP32

The RTOS abstraction (`eriscommon/src/eris_rtos.h`) handles all of this; it is
listed here because it changes how you reason about the firmware.

| | Teensy (ChibiOS) / nRF52 / SAMD21 | ESP32 |
|---|---|---|
| Scheduler | `ERIS_RUN()` starts it (Teensy) | already running — `ERIS_RUN()` spawns threads and **returns** |
| `loop()` | may spin | **must yield**, or the idle task starves and the WDT fires |
| Stack argument | words | **bytes** — tiers are ~4x larger here |
| Critical section | `taskENTER_CRITICAL()` | needs a spinlock; Eris uses one global `portMUX` |
| Cores | single | dual (SMP) — Eris threads are **pinned to one core** |

### Core pinning

Eris's buffers and critical sections were written against a single-core model.
Every Eris thread is therefore pinned to one core (`ERIS_ESP32_CORE`, default
core 1 — the one Arduino's `loopTask` uses; core 0 carries WiFi/BT). This keeps
the concurrency semantics identical to the single-core ports. Unpinning them
would give you true parallelism that the buffer code does not expect.

### Serial baud

`ERIS_SERIAL_BAUD` defaults to **921600**, not the 12 Mbps the Python driver
uses with Teensy. On a classic ESP32 the host talks to a USB-UART bridge
(CP2102/CH340) where 12 Mbps is far out of range. Pass a matching baudrate on
the host:

```python
eris.Eris(port='COM5', baudrate=921600)
```

Native-USB parts (S2/S3/C3) ignore baud, so the default is safe there too.

## Board notes

- Default target is the generic **ESP32 DevKitC** (`esp32dev`).
- `PIN_LED` is **GPIO2** (the DevKitC on-board LED). The generic `esp32` variant
  does not define `LED_BUILTIN` at all. S3/C3 boards typically use an
  addressable LED instead — point `PIN_LED` at an unused GPIO there.
- Single-core parts (S2, C3, C6, H2) are handled: `ERIS_ESP32_CORE` falls back
  to core 0 automatically.
- **ESP8266 is not supported** — its Arduino core is bare-metal with no RTOS.
  `eris_rtos.h` fails the build with an explicit message.

## Building

**Arduino IDE** — install the ESP32 core (verified against 3.3.0 / ESP-IDF
v5.5), select your ESP32 board, open `ErisESP32.ino`. Requires `eriscommon`,
`PacketSerial` and `SerialCommand` in your libraries folder.

**PlatformIO** — `pio run -e ErisESP32`. See the note in `platformio.ini`:
the official `espressif32` platform tracks Arduino-ESP32 2.0.x, while this
flavor is verified against 3.3.0, which needs the `pioarduino` platform fork.
