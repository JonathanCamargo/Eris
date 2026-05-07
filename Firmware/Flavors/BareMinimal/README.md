# BareMinimal

Skeleton template for building new Eris flavors. Boots the RTOS, the heartbeat thread, the SineWave generator, and a serial command parser, with no real sensor module attached.

## Status
Verified (intended as a starting point, not as a deployable firmware)

## Hardware
- **Target board:** Any Arduino-compatible board supported by Eris (Teensy 3.x/4.x, Arduino Due, etc.). `configuration.h` is written for Teensy with `PIN_LED 13` and `PIN_LED_R 28`.
- **RTOS:** Auto-detected ChibiOS (ChRt) or FreeRTOS via `eris_rtos.h`.
- **Sensor / interface:** None. SineWave generates synthetic data only.

## Pin assignments
- `PIN_LED 13` (heartbeat)
- `PIN_LED_R 28` (status)
- FSR/EMG/IMU/SYNC pin macros are defined but no module starts — they are placeholders to copy from when extending.

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand
- PacketSerial

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info string |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump current SineWave buffer to serial |
| `S_F <feat>` | Configure streaming features (only `SINE` is registered) |
| `S_TIME` | Reset the global `t0` timestamp |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/stop sensor threads (stubs) |

## Streaming Features
- `SINE` — synthetic 1 Hz sinewave (`floatSample_t`)

## Sample Session
```
> INFO
> S_F SINE
> S_TIME
> S_ON
... data streams ...
> S_OFF
```

## Notes
Use this directory as the starting point for new flavors: copy it, rename, then add your sensor module (`mysensor.h`/`.cpp` in the `Sensor::` namespace), wire it into `start()` in the `.ino`, register a `TransmitX` command in `serialcommand.cpp`, and add a streaming case in `streaming.cpp`. See the top-level `README.md` "Creating a New Flavor" section for the full procedure.
