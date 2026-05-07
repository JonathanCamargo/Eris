# ErisServoHand

5-DOF robotic hand controller. Maps a 5-value aperture command to 5 PCA9685 servo channels (one per finger), with the thumb inverted.

## Status
Experimental

## Hardware
- **Target board:** Any FreeRTOS-supported board with I2C. The `.ino` includes the FreeRTOS static-allocation callbacks.
- **RTOS:** FreeRTOS (the `.ino` and the `start()` ordering are written for the FreeRTOS path)
- **Sensor / interface:** PCA9685 16-channel PWM driver over I2C (uses the Kasper Skarhoj `PCA9685` library, not the Adafruit one)

## Pin assignments
- `PIN_LED` / `LED_BUILTIN` (heartbeat)
- I2C: `WIRE_PORT Wire` (configurable in `configuration.h`)
- 5 servos on PCA9685 channels 0..4 (thumb=0, then index, middle, ring, pinky)
- Angle range: `MIN_ANGLE 0`, `MAX_ANGLE 90`
- Thumb angle is inverted (`x[0] = 90 - x[0]`) before being written

## Dependencies
- eriscommon
- Arduino_FreeRTOS (or board-specific FreeRTOS port)
- SerialCommand, PacketSerial
- Wire
- PCA9685 (Kasper Skarhoj's library — provides `PCA9685_ServoEval` and `PCA9685::setPWMFreqServo()`)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `HELLO [name]` | Echo greeting (debug) |
| `SHOW_DATA` | Dump synthetic DataGenerator buffer (debug) |
| `X <a0> <a1> <a2> <a3> <a4>` | Move all 5 fingers; `a0`=thumb, `a1..a4`=index..pinky |

The `.ino` header documents three convenience commands (`OPEN`, `CLOSE`, `SHAKA`) but only `X` is currently registered — the high-level gesture commands are TODO.

## Sample Session
```
> INFO
> X 0 0 0 0 0       # fully open
> X 90 90 90 90 90  # fully closed
> X 0 90 90 0 0     # "shaka" (thumb + pinky out, fingers in)
```

## Notes
- No streaming pipeline — this flavor is command-driven only.
- Inherits the FreeRTOS-base scaffolding (heartbeat + DataGenerator + serial command thread) from `ErisFreeRTOSBase`, then layers the `Servos::` namespace on top.
- Per the `.ino` header, `OPEN`/`CLOSE`/`SHAKA` gesture commands are planned but not yet wired into `serialcommand.cpp`.
