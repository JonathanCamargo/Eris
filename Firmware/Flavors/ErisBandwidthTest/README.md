# ErisBandwidthTest

Serial-throughput test firmware. Streams a SineWave buffer and a multi-sample buffer over the wire; pair with `python/Examples/BandwidthTest.py` to measure sustained packet rate.

## Status
Experimental (used as a benchmarking harness, not a sensor flavor)

## Hardware
- **Target board:** Any Eris-compatible board; defaults written for Teensy
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** None (synthetic data only)

## Pin assignments
- `PIN_LED 13`
- `NUMCHANNELS 10` (multi-sample fanout for stress-testing packet size)
- `STREAMING_PERIOD_MS 10` (100 Hz packet rate)
- `TXBUFFERSIZE 16`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SPI (included but unused)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer to console |
| `S_F <feat...>` | Set streaming features |
| `S_TIME` | Reset `t0` |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`. Each `Stream()` cycle pushes both a `floatSample_t` and a `multiSample_t` buffer back-to-back to maximize per-packet payload.

## Sample Session
```
> INFO
> S_F SINE
> S_TIME
> S_ON
... maximum-rate packets stream ...
> S_OFF
```

## Notes
See `python/Examples/BandwidthTest.py` and `python/Examples/BandwidthTest2.ipynb` for measurement scripts.
