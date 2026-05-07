# Eris

Reference firmware flavor: 2-channel analog FSR + sync-input + SineWave + optional SD-card logging on Teensy. The canonical example used to derive most other flavors.

## Status
Verified (baseline reference flavor)

## Hardware
- **Target board:** Teensy 3.x/4.x (uses Teensy SPI and pin numbering)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS via `eris_rtos.h` (FreeRTOS static-allocation callbacks are wired in the `.ino`)
- **Sensor / interface:** Two FSRs on analog pins, one digital sync input, optional SD card

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- `PIN_FSR_0 A0`, `PIN_FSR_1 A1` — `FSR_NUMCHANNELS=2`, sampled at `FSR_FREQUENCY_HZ=1000`
- `PIN_SYNC 29` — digital change-detect input
- `SDCARD true` enables the SD-card recorder (`SDBUFFERSIZE=512`)

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand
- PacketSerial
- SPI, SD (Teensyduino built-ins)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer |
| `FSR` | Dump FSR buffer (channel 0) |
| `SYNC` | Dump sync-input change buffer |
| `S_F <feat...>` | Set streaming features |
| `S_TIME` | Reset `t0` |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill sensor threads (stubs) |
| `SD_REC <name>` | Begin SD recording with trial name |
| `SD_NREC` | Stop SD recording |

## Streaming Features
`S_F` accepts: `SINE`, `FSR`, `SYNC`.

## Sample Session
```
> INFO
> S_F FSR SYNC
> S_TIME
> SD_REC trial01
> S_ON
... FSR + sync samples stream and write to SD ...
> S_OFF
> SD_NREC
```

## Notes
This is the cleanest "everything that's standard" flavor. Custom flavors should diverge from here rather than from BareMinimal if they need FSR/SD/sync bundled in.
