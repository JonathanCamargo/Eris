# ErisADS131

Low-power 24-bit EMG firmware using the TI ADS131 over SPI. Streams 8-channel EMG (currently configured for 3 active channels).

## Status
Experimental (per `Flavors/README.txt`, marked for update)

## Hardware
- **Target board:** Teensy 3.x (auto-detected `__MK2x__`/`__MK6x__` macros) or Arduino Due (`__SAM3X8E__`)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** TI ADS131 ADC over SPI

## Pin assignments
- `PIN_LED 13` (Teensy)
- ADS131 control pins:
  - `PIN_ADC0_SS 10`, `PIN_ADC0_DRDY 5`
  - `PIN_ADC0_START 24`, `PIN_ADC0_RESET 25`, `PIN_ADC0_PWDN 26`
  - `PIN_ADC0_TESTN 27`, `PIN_ADC0_TESTP 28`
  - GPIOs: `PIN_ADC0_GPIO1 32`, `PIN_ADC0_GPIO2 8`, `PIN_ADC0_GPIO3 7`, `PIN_ADC0_GPIO4 6`
- `EMG_NUMCHANNELS 3`
- Buffer: `TXBUFFERSIZE 16`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SPI
- ADS131

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `EMG` | Dump current EMG buffer |
| `S_F <feat...>` | Set streaming features |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill threads (stubs) |
| `SD_REC` | Start ADC collection (note: this command is repurposed here) |
| `SD_NREC` | Stop ADC collection |

## Streaming Features
`S_F` accepts: `EMG`.

## Sample Session
```
> INFO
> SD_REC
> S_F EMG
> S_ON
... 24-bit EMG samples stream ...
> S_OFF
> SD_NREC
```

## Notes
- `SD_REC`/`SD_NREC` are wired to `ADS131::startCollecting()` / `stopCollecting()` rather than SD-card recording, since this flavor has no SD module.
- Authored with Will Flanagan ("ossip and wf" in the firmware string). Firmware version reads `v2.0`.
