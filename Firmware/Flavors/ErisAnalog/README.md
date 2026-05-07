# ErisAnalog

Built-in-ADC EMG flavor: configurable analog channels sampled at 1 kHz, plus an FSR channel.

## Status
Verified (per `Flavors/README.txt`: "Updated and verified, renamed from Analog10k")

## Hardware
- **Target board:** Teensy 3.x/4.x (uses Teensy `IntervalTimer`)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** Built-in ADC, no external chip

## Pin assignments
- `PIN_LED 13`
- `PINS_ANALOG A0,A1` — `ANALOG_NUMCHANNELS 2` (expandable up to A0..A7)
- `PIN_FSR A20` — `FSR_NUMCHANNELS 1` @ 100 Hz
- `ANALOG_PERIOD_US 1000` (1 kHz analog sampling)

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- IntervalTimer (Teensyduino)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer |
| `A` | Dump current Analog buffer |
| `TIME0` | Reset `t0` |
| `S_F <feat...>` | Set streaming features |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `Analog` (case-sensitive — note the capital A).

## Sample Session
```
> INFO
> S_F Analog
> TIME0
> S_ON
... 2-channel analog samples stream at 1 kHz ...
> S_OFF
```

## Notes
- This flavor uses `TIME0` for time sync rather than the more common `S_TIME`.
- Despite the file's `// Flavor: ErisNextFlexAnalog` header comment, this is the standalone analog-EMG flavor (NextFlex variants live in their own directories).
