# ErisEMGPill

Surface EMG from analog **EMG Pill** front-ends (single-channel amplifiers with an
analog `OUT` biased at mid-supply), sampled on the board's built-in ADC.

Default: **2 channels @ 500 Hz** on a **Seeed XIAO ESP32S3**.

## Status

🟡 Compile-verified on XIAO ESP32S3 (esp32 core 3.3.11). **Not yet run on hardware.**

## Hardware

- **Board:** Seeed XIAO ESP32S3. Any Eris board with free ADC pins works; nothing here is S3-specific.
- **Front-end:** one EMG Pill per channel, powered from **3V3**.
- **Electrodes:** three per pill (IN+, IN−, REF), as the pill's documentation specifies.

### Wiring

| Signal | XIAO ESP32S3 pin | Config macro |
|---|---|---|
| Pill 0 `OUT` | A0 / D0 (GPIO 1) = `ERIS_ADC(0)` | `PINS_EMG` |
| Pill 1 `OUT` | A1 / D1 (GPIO 2) = `ERIS_ADC(1)` | `PINS_EMG` |
| Pill `VCC` | 3V3 | — |
| Pill `GND` | GND | — |

- Power the pills from **3V3, never 5 V**. `OUT` can swing to the rail, and the ADC pin can't take more than 3.3 V.
- Both pins are on ADC1, so `analogRead` keeps working with WiFi on. `emgpill.cpp` checks this at compile time with `ERIS_PIN_ASSERT_ADC1`.
- For EMG, run the board from a battery or an isolated USB supply. **Never** have electrodes on a person while the board is on a mains-powered PC without isolation.

## Quick start

1. Flash, then open the Serial Monitor or Plotter at 921600 (USB CDC on the S3 ignores the baud).
2. `EMG.ch0` and `EMG.ch1` stream straight away. At rest each sits near 1.65 V and moves when you contract the muscle.
3. To record, use the Python driver, which switches to binary on connect:

```python
from eris.eris import Eris
e = Eris(['EMG'], port='COM5')     # layout comes from DESC
e.start()
out = e.read()
for s in out['D'][0]['EMG']:
    print(s.timestamp, s.ch0, s.ch1)
```

## Streaming

`S_F` accepts `EMG` and `SINE`. The board boots streaming `EMG` in ASCII mode.

```
DESC EMG timestamp:f32:t,ch0:f32,ch1:f32
```

| Field | Meaning |
|---|---|
| `timestamp` | ms since `t0` |
| `chN` | Volts at pill N's `OUT`. On ESP32 this is `analogReadMilliVolts()`, which uses the eFuse calibration |

The signal is **raw** and includes the mid-supply DC offset. Band-pass it on the
host (e.g. 20–250 Hz), then rectify and low-pass for an envelope.

> Text mode shows only the newest sample, at about 20 Hz. Use it as a monitor,
> not to record data. Record with `S_MODE BIN` or the Python driver.

## Serial commands

The standard Eris set (`INFO`, `S_F`, `S_ON`/`S_OFF`, `S_MODE`, `DESC`, `S_TIME`, …) plus:

| Command | Description |
|---|---|
| `EMG` | Dump the buffered EMG samples as text |

## Configuration (`configuration.h`)

| Macro | Default | Notes |
|---|---|---|
| `EMG_NUMCHANNELS` | 2 | The descriptor follows automatically (`ERIS_DESCRIBE_CHANNELS`) |
| `PINS_EMG` | `ERIS_ADC(0), ERIS_ADC(1)` | Must list exactly `EMG_NUMCHANNELS` pins. Add an `ERIS_PIN_ASSERT_ADC1` line in `emgpill.cpp` for each new pin |
| `EMG_HZ` | 500 | Must divide 1000; 1000 is the ceiling |
| `EMG_TXBUFFERSIZE` | 16 | Max samples per packet (about 5 are waiting per 10 ms stream tick at 500 Hz) |

## Files

| File | Contents |
|---|---|
| `emgpill.cpp` | ADC setup and the `ERIS_SENSOR` sampling thread |
| `customtypes.h` | `EMGPillSample_t` and its descriptor |
| `streaming.cpp` | `S_F EMG` / `SINE` |
| `serialcommand.cpp` | `EMG` dump command |

## Dependencies

eriscommon, SerialCommand and PacketSerial. The ESP32 core provides FreeRTOS, so there is no RTOS lib_dep.
