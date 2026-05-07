# ErisServoINA

`ErisServo` plus an INA219 current/voltage/power sensor on the same I2C bus. Streams servo current draw at 100 Hz alongside servo motion control.

## Status
Experimental

## Hardware
- **Target board:** Any I2C-capable Arduino board
- **RTOS:** Auto-detected ChibiOS or FreeRTOS (`.ino` includes FreeRTOS static-allocation callbacks)
- **Sensor / interface:** PCA9685 servo driver + INA219 current sensor, both over I2C

## Pin assignments
- `PIN_LED 13`
- I2C: default `Wire`
- Servos (same as `ErisServo`):
  - `NUM_SERVOS 16`
  - `SERVO_MIN_PULSE 150`, `SERVO_MAX_PULSE 600`
  - `SERVO_MIN_ANGLE 0`, `SERVO_MAX_ANGLE 180`
  - `PCA9685_I2C_ADDR 0x40`
  - Smooth move: `SERVO_SMOOTH_SPEED 180 deg/s`, `SERVO_SMOOTH_DECEL_DEG 6.0`, `SERVO_SMOOTH_LOOP_MS 5` (200 Hz)
- INA219:
  - `INA219_I2C_ADDR 0x40` (collides with PCA9685 default — see Notes)
  - `INA219_SAMPLE_RATE_HZ 100`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- Wire
- Adafruit_PWMServoDriver
- Adafruit_INA219

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer |
| `INA` | Dump INA219 buffer (current_mA + voltage + power) |
| `X <ch> <angle>` or `X <a0..a15>` | Move servo(s) immediately |
| `Y <ch> <angle>` or `Y <a0..a15>` | Smooth move |
| `S_TIME` | Reset `t0` |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `INA`.

## Sample Session
```
> INFO
> S_F INA
> S_TIME
> S_ON
> Y 0 90      # smoothly move servo 0 while watching current draw
... INA219 current samples stream at 100 Hz ...
> S_OFF
```

## Notes
- **I2C address conflict:** `PCA9685_I2C_ADDR` and `INA219_I2C_ADDR` both default to `0x40` in `configuration.h`. In a real wiring you must change the address-jumper on at least one of the two boards and edit the corresponding macro before building.
