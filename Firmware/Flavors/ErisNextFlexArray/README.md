# ErisNextFlexArray

NextFlex 8-channel SPI EMG array with software-selectable negative electrode. Per-channel buffers (rather than a single multi-channel buffer) padded with NaN for missing samples.

## Status
Experimental (variant of `ErisNextFlex`)

## Hardware
- **Target board:** Teensy 3.x/4.x (uses fastdualbuffer + TimerOne)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** ADS1256 24-bit ADC over SPI (8-channel array); FSR on analog; UART-driven negative-electrode selector

## Pin assignments
- `PIN_LED 13`
- ADS1256: `PIN_ADC0_SS 6`, `PIN_ADC0_DRDY 28`
- `PIN_FSR A1` — `NUMFSRCHANNELS 1` @ 100 Hz
- `NUMEMGCHANNELS 8`
- Negative-electrode selector UART: `EMG_SERIAL Serial3` (pins 7 RX3 / 8 TX3)
- `SDCARD true`

## Dependencies
- eriscommon
- FastDualBuffer
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SPI, SD
- ADS1256
- TimerOne

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `FSR` | Dump named buffer |
| `NEG <n>` | Select negative-electrode lead via the selector UART |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |
| `SD_REC <name>` / `SD_NREC` | Start/stop SD recording |

## Streaming Features
`S_F` accepts: `SineWave`, `EMG`, `FSR` (note: keys are case-sensitive substring matches; `SINE` will not match `SineWave` here).

## Sample Session
```
> INFO
> NEG 3
> SD_REC array_trial1
> S_F EMG FSR
> S_ON
... 8-channel EMG + FSR stream and write to SD ...
> S_OFF
> SD_NREC
```

## Notes
- The streaming `EMG()` function pads each channel buffer with `NAN` to reach `TXBUFFERSIZE` so all 8 channels emit fixed-size payloads even when individual channels miss samples.
- `NEG` reconfigures the differential reference electrode by sending a command over `EMG_SERIAL`.
