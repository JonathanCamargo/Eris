# ErisNextFlex

NextFlex amplifier flavor: 2-channel EMG via SPI ADS1256, 1-channel FSR, plus a temperature/impedance (ETI) telemetry channel from the amplifier cable's auxiliary serial ports.

## Status
Updated, awaiting verification (per `Flavors/README.txt`: "Updated waiting verification with real circuit")

## Hardware
- **Target board:** Teensy 3.x/4.x (auto-detected) or Arduino Due
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** ADS1256 (Texas Instruments 24-bit ADC) over SPI; FSR on analog pin; ETI auxiliary data on two hardware UARTs

## Pin assignments
- `PIN_LED 13`
- ADS1256: `PIN_ADC0_SS 6`, `PIN_ADC0_DRDY 28`
- `PIN_FSR A1` — 1 channel @ `FSR_FREQUENCY_HZ 100`
- `EMG_NUMCHANNELS 2`, `EMG_TXBUFFERSIZE 10`
- ETI auxiliary UARTs: `ETI_SERIAL0 Serial4`, `ETI_SERIAL1 Serial3`
- `SDCARD true` (SD logging enabled by default)

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SPI, SD
- ADS1256_4CH
- TimerOne

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `FSR` / `ETI` | Dump named buffer to serial |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |
| `SD_REC <name>` / `SD_NREC` | Start/stop SD card recording |

`S_TIME` is not registered in this flavor.

## Streaming Features
`S_F` accepts: `SINE`, `EMG`, `FSR`, `ETI`.

## Sample Session
```
> INFO
> SD_REC walk_trial1
> S_F EMG FSR ETI
> S_ON
... data streams + writes to SD ...
> S_OFF
> SD_NREC
```

## Notes
- ETI = "EMG Temperature/Impedance": metadata about each electrode read out of the amplifier cable's UARTs.
- The `Eris.h` for this flavor uses `eris_thread_ref_treadSerial` (no space before `readSerial`) — purely a stylistic quirk; the macro `eris_thread_ref_t` still expands correctly.
