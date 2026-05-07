# ErisServo

PCA9685 servo control flavor. Drives up to 16 servos via I2C using the Adafruit PWM Servo Driver. Built on `ErisMinimal`'s portable foundation (no Teensy-specific code).

## Status
Verified working (per project memory)

## Hardware
- **Target board:** Any I2C-capable Arduino board (Teensy, Due, SAMD21 — verified across `eris_rtos.h` targets)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS via `eris_rtos.h`. The `.ino` does not include the FreeRTOS static-allocation callbacks (Teensy/ChibiOS path is the verified one).
- **Sensor / interface:** PCA9685 16-channel PWM driver over I2C @ `0x40`

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
