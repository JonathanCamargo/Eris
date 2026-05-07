# ErisNextFlexAnalog

NextFlex variant that reads EMG straight from the amplifier cable on the built-in ADC (instead of via SPI ADS1256). One EMG channel + one FSR channel + ETI telemetry.

## Status
Experimental (variant of the verified `ErisNextFlex`)

## Hardware
- **Target board:** Teensy 3.x/4.x (uses `IntervalTimer`)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** Built-in ADC for EMG and FSR; one auxiliary UART for ETI

## Pin assignments
- `PIN_LED 13`
- `PIN_EMG A21` — `EMG_NUMCHANNELS 1` @ `EMG_PERIOD_US 1000` (1 kHz)
- `PIN_FSR A20` — `FSR_NUMCHANNELS 1` @ 100 Hz
- ETI auxiliary UART: `ETI_SERIAL0 Serial2`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- IntervalTimer

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `FSR` / `ETI` | Dump named buffer |
| `TIME0` | Reset `t0` |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `EMG`, `ETI`, `FSR`.

## Sample Session
```
> INFO
> S_F EMG FSR ETI
> TIME0
> S_ON
... 1 kHz EMG + 100 Hz FSR + low-rate ETI stream ...
> S_OFF
```

## Notes
- No SD card support — pure streaming variant.
- Uses legacy `TIME0` for time sync rather than `S_TIME`.
- "ETI" = electrode temperature/impedance from the amplifier cable's UART telemetry.
