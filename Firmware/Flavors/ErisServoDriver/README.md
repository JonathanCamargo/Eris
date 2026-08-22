# ErisServo

PCA9685 servo control flavor. Drives up to 16 servos via I2C using the Adafruit PWM Servo Driver. Built on `ErisMinimal`'s portable foundation (no Teensy-specific code).

## Status
Verified working (per project memory)

## Hardware
- **Target board:** Teensy (ChibiOS) and **ESP32** (FreeRTOS) are both supported from
  one source tree; other I2C-capable boards (Due, SAMD21, nRF52) fall through to a
  generic default. See *Board configuration* below.
- **RTOS:** Auto-detected ChibiOS or FreeRTOS via `eris_rtos.h`. The `.ino` does not include the FreeRTOS static-allocation callbacks (they live in `eriscommon`).
- **Sensor / interface:** PCA9685 16-channel PWM driver over I2C @ `0x40`

## Board configuration

`configuration.h` detects the target and selects the pin map. Nothing else in the
flavor is board-specific — `servos.cpp` calls `ERIS_I2C_BEGIN()` rather than
`Wire.begin()` directly.

| | ESP32 | Teensy | Other |
|---|---|---|---|
| `PIN_LED` | `2` (DevKitC LED; no `LED_BUILTIN` on the generic variant) | `13` | `LED_BUILTIN` |
| `ERIS_I2C_BEGIN()` | `Wire.begin(21, 22)` | `Wire.begin()` | `Wire.begin()` |
| `ERIS_SERIAL_BAUD` | `921600` (UART-bridge ceiling) | `115200` (USB CDC ignores it) | `115200` |

Detection keys off `ESP32` / `TEENSYDUINO`. To force a target, define
`ERIS_BOARD_ESP32`, `ERIS_BOARD_TEENSY` or `ERIS_BOARD_OTHER` before
`configuration.h`. Every individual macro is `#ifndef`-guarded, so you can
override just one (e.g. `-DPIN_I2C_SDA=25`) without taking the whole block.

**ESP32 wiring:** PCA9685 `SDA`→GPIO21, `SCL`→GPIO22, `VCC`→3V3, `GND`→GND.
Any free GPIO works — the ESP32 routes I2C through the GPIO matrix — so change
`PIN_I2C_SDA` / `PIN_I2C_SCL` to match your board. Servo power (`V+`) must come
from a separate supply with its ground tied to the ESP32's; 16 servos will brown
out a USB rail.

**Teensy** pins I2C in hardware, so there is nothing to pass to `Wire.begin()`.
Use `Wire.setSDA()` / `Wire.setSCL()` before `Servos::start()` for an alternate bus.

Both targets are compile-verified (ESP32 against arduino-esp32 3.3.11 / ESP-IDF
v5.5.5; Teensy 3.6 against Teensyduino 1.59 + ChRt). Neither has been run on
hardware since the port.

## Pin assignments
- `PIN_LED 13` (heartbeat)
- I2C: default `Wire` (SDA/SCL for the target board)
- `NUM_SERVOS 16`
- `SERVO_MIN_PULSE 150`, `SERVO_MAX_PULSE 600` (Adafruit PWM ticks out of 4096)
- `SERVO_MIN_ANGLE 0`, `SERVO_MAX_ANGLE 180`
- `PCA9685_I2C_ADDR 0x40`
- Smooth-move profile: `SERVO_SMOOTH_SPEED 180.0` deg/s, `SERVO_SMOOTH_DECEL_DEG 6.0`, `SERVO_SMOOTH_LOOP_MS 5` (200 Hz tick)

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- Wire (I2C built-in)
- Adafruit_PWMServoDriver

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer |
| `X <ch> <angle>` or `X <a0> <a1> ... <a15>` | Move single servo or all 16 immediately |
| `Y <ch> <angle>` or `Y <a0> <a1> ... <a15>` | Smooth (rate-limited, ease-out) move |
| `S_TIME` | Reset `t0` |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE` only.

## Sample Session
```
> INFO
> X 0 90       # snap servo 0 to 90 deg
> Y 1 30       # smoothly move servo 1 to 30 deg
> X 0 30 60 90 120 150 0 0 0 0 0 0 0 0 0 0   # all 16 channels at once
> S_F SINE
> S_ON
... sinewave streams while servos hold position ...
> S_OFF
```

## Notes
- The smooth-move thread runs continuously at 200 Hz and applies an ease-out profile within `SERVO_SMOOTH_DECEL_DEG` degrees of the target.
- Both Teensy and Seeeduino XIAO targets have been used to validate this flavor.
