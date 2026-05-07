# ErisTapok2

Updated Tapok flavor with SD-card recording and a second EMG transmit command. Same Tapok-over-CAN sensor architecture as `ErisTapok` but with a heavier feature footprint.

## Status
Experimental

## Hardware
- **Target board:** Teensy 3.x/4.x with FlexCAN
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** Tapok wireless EMG over CAN (`Can0`); FSR on analog; digital sync input

## Pin assignments
- `PIN_LED 13`
- CAN: `CANTAPOK Can0`
- `EMG_NUMCHANNELS 8`, `EMG_TXBUFFERSIZE 24`, `EMG_GAIN 2`
- `PIN_FSR A1` — `FSR_NUMCHANNELS 1` @ `FSR_FREQUENCY_HZ 1000`
- `PIN_SYNC 28`
- `SDCARD true` (SD logging enabled by default)

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SD
- FlexCAN
- tapok
- FeatureExtractor (per top-level README's library table)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `EMG` / `EMG2` / `FSR` / `SYNC` | Dump named buffer |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` / `KILL` | Start/kill threads (stubs) |
| `SD_REC <name>` / `SD_NREC` | Start/stop SD recording |

`S_TIME` is not registered.

## Streaming Features
`S_F` accepts: `SINE`, `FSR`. (Note: `EMG` is parsed but its streaming function is currently commented out in `streaming.cpp`.)

## Sample Session
```
> INFO
> SD_REC tapok_trial1
> S_F FSR
> S_ON
... FSR samples stream and write to SD ...
> S_OFF
> SD_NREC
```

## Notes
- Adds SD recording compared to `ErisTapok`.
- The `EMG2` transmit command exists but the EMG streaming function is currently no-op (`streamfnc[]` assignment is commented out).
- `boolSample_t` is used for sync (vs. `uint8_tSample_t` in `ErisTapok`).
