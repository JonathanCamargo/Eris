# ErisBiom2

Second-generation biomechanics flavor with a leaner feature-extraction API. Same 6-channel Biom acquisition as `ErisBiom`, but with a redesigned regression/classification command surface (`REG`, `CLASS`).

## Status
Experimental

## Hardware
- **Target board:** Teensy 3.x/4.x
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** Same as `ErisBiom` (6-channel analog Biom main unit + sync)

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- `PIN_BIOM A0,A1,A2,A3,A4,A5` — `BIOM_NUMCHANNELS 6` @ `BIOM_FREQUENCY_HZ 1000`
- `PIN_SYNC 29`
- ADC SPI: `PIN_ADC0_SS 6`, `PIN_ADC0_DRDY 28`
- Features: `FEATS_NUM 9`, `FEATS_NUMCLASSLOCATIONS 6`

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
| `S_TIME` | Reset `t0` |
| `SINE` / `BIOM` / `GAIT` / `FEAT <idx>` | Dump buffers / feature vector |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `F_ON` / `F_OFF` | Enable/disable feature extraction |
| `REG <win> <inc> <mode>` | Update regression window, increment, ambulation mode |
| `CLASS <win> <gait>` | One-shot classification feature query |
| `START` / `KILL` | Start/kill threads (stubs) |
| `test` | Debug ping (sends "hola James") |

## Streaming Features
`S_F` accepts: `SineWave`, `BIOM`, `Gait`.

## Sample Session
```
> INFO
> S_TIME
> F_ON
> REG 250 50 0
> S_F BIOM Gait
> S_ON
... biom + gait stream while regression features are computed in the background ...
> CLASS 100 2
... single classification packet returned ...
> S_OFF
```

## Notes
This flavor reorganizes the `ErisBiom` command set: regression and classification are now controlled via `REG`/`CLASS` rather than the older `F_R`/`F_MASK`/`F_WIN`/`F_IDX` quartet. `F_MASK` exists in the source (`changeMask` function) but is not registered as a serial command.
