# ErisADS1299

8-channel 24-bit EMG/ECG bioamplifier firmware. Streams ADS1299 samples over SPI at the chip's native rate, plus a single FSR channel.

## Status
Verified (per project memory and `Flavors/README.txt`)

## Hardware
- **Target board:** Teensy 3.x/4.x (`#define TEENSY` is hard-set in `configuration.h`)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS via `eris_rtos.h`
- **Sensor / interface:** Texas Instruments ADS1299 over SPI; FSR on analog pin

## Pin assignments
- `PIN_LED 14`, `PIN_ERROR 12`
- ADS1299: `PIN_EMG_ADC0_SS 8`, `PIN_EMG_ADC0_DRDY 7`, `PIN_EMG_RESET 6`
- `PIN_FSR A13` — single FSR @ `FSR_FREQUENCY_HZ=1000`
- `PIN_SYNC 28` (sync module currently commented out in the `.ino`)
- `EMG_NUMCHANNELS 8`, `EMG_TXBUFFERSIZE 24`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SPI
- ADS1299
- Filters
- FeatureExtractor

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `FSR` / `SYNC` | Dump named buffer to serial |
| `S_F <feat...>` | Set streaming features |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill sensor threads (stubs) |
| `SD_REC <name>` / `SD_NREC` | Start/stop SD recording (only if `SDCARD` enabled) |

Note: `S_TIME` is not registered in this flavor.

## Streaming Features
`S_F` accepts: `SINE`, `EMG`, `FSR`.

## Sample Session
```
> INFO
> S_F EMG FSR
> S_ON
... 8-ch EMG + FSR samples stream ...
> S_OFF
```

## Notes
- Sync module is included in source but `Sync::start()` is commented out in `ErisADS1299.ino`.
- Designed for serial-only operation (no SD by default in the verified config); set `#define SDCARD true` in `configuration.h` to enable.
