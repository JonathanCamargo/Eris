# ErisBiom

Biomechanics motion-capture firmware: 6-channel analog Biom acquisition at 1 kHz with on-board feature extraction for regression and gait classification.

## Status
Experimental (heavy ML/feature-extraction pipeline; used internally for prosthesis control research)

## Hardware
- **Target board:** Teensy 3.x/4.x (uses `IntervalTimer`)
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** 6-channel analog input (e.g. Biometrics Ltd. goniometers / EMG main unit) plus a sync digital input

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- `PIN_BIOM A0,A1,A2,A3,A4,A5` — `BIOM_NUMCHANNELS 6` @ `BIOM_FREQUENCY_HZ 1000`
- `BIOM_HIP_GON_CHAN 0` (channel index used as hip goniometer for gait detection)
- `PIN_SYNC 29`
- ADC SPI: `PIN_ADC0_SS 6`, `PIN_ADC0_DRDY 28`
- Feature settings: `FEATURES_REGRESSION_WINDOW_MS 250`, `FEATURES_REGRESSION_PERIOD_MS 50`, `FEATURES_CLASSIFICATION_WINDOW_MS 100`, `FEATURES_NUMREG 3`, `FEATURES_NUMCLASS 4`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- IntervalTimer
- FeatureExtractor

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `TPAC <0/1>` | Wrap text prints as T-packets |
| `ON` / `OFF` | LED on/off |
| `SINE` / `BIOM` / `GAIT` / `SYNC` | Dump named buffer to serial |
| `S_TIME` | Reset `t0` |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `F_ON` / `F_OFF` | Enable/disable on-board feature extraction |
| `F_R` | Register a feature helper |
| `F_MASK` | Configure which features are computed |
| `F_WIN` | Change window size |
| `F_IDX` | Change index of a feature-helper array |
| `F_CLASS` | Compute classification features once |
| `F_INFO` | Print feature-helper status |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SineWave`, `BIOM`, `Gait`.

## Sample Session
```
> INFO
> S_TIME
> F_MASK 0 0 0 11111111
> F_ON
> S_F BIOM Gait
> S_ON
... biom + gait + features stream ...
> S_OFF
> F_OFF
```

## Notes
The feature-extraction subsystem must be configured (masks set with `F_MASK`) before enabling with `F_ON`, otherwise no features are computed. See the comment block at the top of `serialcommand.cpp` for the protocol.
