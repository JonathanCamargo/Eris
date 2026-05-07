# ErisDCMotor

DC motor open-loop driver with potentiometer position feedback. Two FSRs and a SineWave generator are wired in for completeness.

## Status
Experimental

## Hardware
- **Target board:** Teensy 3.x/4.x (uses Teensy pin numbering)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** H-bridge DC motor (2 PWM pins), potentiometer on analog pin, FSRs on analog pins

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- Motor H-bridge: `PIN_MOT_0_A 0`, `PIN_MOT_0_B 1`
- `PIN_POT_0 A9` — `POT_NUMCHANNELS 1` @ `POT_FREQUENCY_HZ 100`
- `PIN_FSR_0 A0`, `PIN_FSR_1 A1` — `FSR_NUMCHANNELS 2` @ `FSR_FREQUENCY_HZ 1000`
- `PIN_SYNC 29`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- PID_v1 (Brett Beauregard)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer |
| `POT` | Dump potentiometer buffer |
| `MOT_F` | Drive motor forward |
| `MOT_B` | Drive motor backward |
| `MOT_STOP` | Brake motor |
| `MOT_OFF` | Idle motor (coast) |
| `S_F <feat...>` | Set streaming features |
| `S_TIME` | Reset `t0` |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `POT`.

## Sample Session
```
> INFO
> S_F POT
> S_TIME
> S_ON
> MOT_F
... potentiometer angle streams while motor turns ...
> MOT_STOP
> S_OFF
```

## Notes
PID_v1 is included for closed-loop control but the current `serialcommand.cpp` only exposes open-loop direction commands. Streaming `SineWave()` is currently a no-op (commented out in `streaming.cpp`).
