# ErisTapok

Tapok wireless EMG receiver. Collects 8-channel EMG samples streamed over CAN bus from a Tapok transmitter, plus FSR + sync.

## Status
Experimental (per `Flavors/README.txt`: "Untouched... awaiting update")

## Hardware
- **Target board:** Teensy 3.x/4.x with FlexCAN (uses `Can1`)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** Tapok wireless EMG node communicating over CAN @ 1 Mbps; FSR on analog; digital sync input

## Pin assignments
- `PIN_LED 13`, `PIN_ERROR 12`
- CAN: `CANTAPOK Can1` @ 1 Mbps
- `EMG_NUMCHANNELS 8`, `EMG_TXBUFFERSIZE 24`
- `PIN_FSR A2` — `FSR_NUMCHANNELS 1` @ `FSR_ADC_FREQUENCY_HZ 100`
- `PIN_SYNC 28`, `SYNC_FREQUENCY_US 5000000` (0.2 Hz)
- `STREAMING_PERIOD_MS 10`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- FlexCAN
- tapok (custom CAN protocol library)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `FSR` / `SYNC` | Dump named buffer |
| `TIME0` | Reset `t0` |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINEWAVE`, `EMG`, `FSR`, `SYNC` (note: matches against `SINEWAVE`, not `SINE`).

## Sample Session
```
> INFO
> TIME0
> S_F EMG FSR
> S_ON
... Tapok EMG samples (received over CAN) + FSR stream ...
> S_OFF
```

## Notes
- `Sync()` streaming function is currently commented out in `streaming.cpp` even though `SYNC` is parsed.
- No SD card support in this flavor (see `ErisTapok2` for the SD-enabled version).
- Uses legacy `TIME0` rather than `S_TIME`.
