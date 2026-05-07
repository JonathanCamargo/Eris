# ErisNextFlexArrayAnalog

NextFlex 8-channel array variant using built-in analog EMG (no SPI ADS1256). Software-selectable negative electrode via UART. No SD card.

## Status
Experimental (variant of `ErisNextFlexArray`)

## Hardware
- **Target board:** Teensy 3.x/4.x (uses Teensy `A0..A7` analog block)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** Built-in ADC for 8 EMG channels + 1 FSR; UART negative-electrode selector

## Pin assignments
- `PIN_LED 13`
- `PINS_EMG = {A0,A1,A2,A3,A4,A5,A6,A7}` — `EMG_NUMCHANNELS 8` @ `EMG_PERIOD_US 1000` (1 kHz)
- `PIN_FSR A8` — `FSR_NUMCHANNELS 1` @ 100 Hz
- Negative-electrode selector UART: `EMG_SERIAL Serial1`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `FSR` | Dump named buffer |
| `NEG <n>` | Select negative-electrode lead via the selector UART |
| `TIME0` | Reset `t0` |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `EMG`, `FSR`.

## Sample Session
```
> INFO
> NEG 3
> S_F EMG FSR
> TIME0
> S_ON
... 8-channel analog EMG + FSR stream ...
> S_OFF
```

## Notes
- Combines NextFlexArray's 8-channel architecture with NextFlexAnalog's built-in-ADC sampling. Trade-off: lower resolution than the SPI array but no external chip needed.
- Uses legacy `TIME0` for time sync.
- No SD card support in this variant.
