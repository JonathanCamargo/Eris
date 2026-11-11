# ErisFSRCalibrator

A rig for calibrating a force-sensitive resistor (FSR). A stepper on a lead screw
presses a probe onto the FSR, and the FSR sits on a load cell that measures the
true force. The firmware streams both readings, taken on the same tick:

- **voltage**: the FSR voltage-divider output, in V
- **force**: the load-cell force, in N
- **position**: where the axis was, in mm (so you can also fit force against displacement)

Each streamed row is one matched (voltage, force) pair, which is what a
calibration fit needs.

```
        stepper + lead screw
               |
             probe
               v
      ======= FSR =======
      ===== load cell ===
             base
```

## Status

🟡 Compile-verified on Teensy 3.6 (ChibiOS), Seeeduino XIAO / SAMD21 (FreeRTOS)
and ESP32 (core 3.3.x). **Not yet run on hardware.**

## Hardware

- **Board:** any Eris-supported board. Motion runs on an RTOS thread, not a timer.
- **Stepper driver:** A4988 (Pololu or clone). The motor supply is separate from the MCU (8–35 V), with grounds tied together.
- **Load cell amplifier:** HX711 breakout plus a 4-wire bar or button load cell.
- **FSR:** any two-terminal FSR plus a fixed resistor (10 kΩ to start).

### Wiring

| Signal | Teensy / SAMD default | ESP32 default | Config macro |
|---|---|---|---|
| A4988 `STEP` | 2 | 25 | `PIN_A4988_STEP` |
| A4988 `DIR`  | 3 | 26 | `PIN_A4988_DIR` |
| A4988 `EN`   | 4 | 27 | `PIN_A4988_EN` (active LOW) |
| HX711 `DOUT` | 5 | 18 | `PIN_HX711_DOUT` |
| HX711 `SCK`  | 6 | 19 | `PIN_HX711_SCK` |
| FSR divider tap | `ERIS_ADC(0)` (A0) | `ERIS_ADC(0)` (GPIO 36) | `PIN_FSR` |

A4988 notes:
- Tie **RESET to SLEEP**. If you don't, the driver does nothing.
- **MS1/MS2/MS3 set the microstepping in hardware.** `A4988_MICROSTEPS` must match them, or every distance is off by that ratio. All three HIGH gives 16.
- Set the current limit with the trimpot *before* the first run: `Vref = I_max × 8 × R_sense` (Pololu R_sense = 0.068 Ω, so 0.5 A ≈ 0.27 V).

HX711 notes:
- The `RATE` pin selects 10 SPS (GND, the usual default) or 80 SPS (VCC). Set `LOADCELL_SPS` to match. It only affects the banner and how long a tare takes.
- Load cell wires: E+/E− are excitation, A+/A− are signal. If force reads negative when pressed, swap A+/A−, or just calibrate. A negative scale works too.

FSR divider:

```
  3V3 --[ FSR ]--+--[ 10k ]-- GND
                 |
              ERIS_ADC(0)
```

Voltage **rises** with force: `V = 3.3 · R / (R_fsr + R)`, so
`R_fsr = R · (3.3 − V) / V`. The firmware streams only `V`. Compute resistance
or conductance (`1/R_fsr`, roughly linear in force) on the host from
`FSR_VREF` and `FSR_R_FIXED`. Power the divider from **3V3**, never 5 V.

## Quick start

1. Make sure nothing is pressing on the load cell, then power up. The first `LOADCELL_AVG_SAMPLES` conversions tare it automatically (2 s at 10 SPS).
2. Open the Serial Monitor or Plotter at 115200. `FSR.voltage`, `FSR.force` and `FSR.position` stream straight away.
3. **Calibrate the load cell once:** put a known weight on it and send `LCAL 4.903` (for 500 g: newtons = kg × 9.80665). The firmware prints the counts/N. Paste that value into `LOADCELL_COUNTS_PER_N` and set `LOADCELL_CALIBRATED 1`, and the next boot is calibrated. Until then, `force` is in tared counts and **the overload guard is off**.
4. Jog the probe down onto the FSR in small steps and record:

```
G0 X0.5
G0 X1.0
G0 X1.2 F30
...
```

## Overload guard

A lead screw can easily crush a small load cell, so the firmware watches the
force. Once it exceeds `CAL_FORCE_LIMIT_N` (default 50 N), the axis **stops
dead**, and moves in the pressing direction (`CAL_PRESS_DIR`) are refused until
the force drops below 90% of the limit. Moves that back off are always
accepted. The guard also blocks pressing if the HX711 stops converting, and from
boot until the first reading arrives.

Set the limit below the rating of both the load cell *and* the FSR. The guard
only exists once the load cell is calibrated. A limit in newtons means nothing
without a scale.

## Serial commands

The standard Eris set (`INFO`, `S_F`, `S_ON`/`S_OFF`, `S_MODE`, `DESC`, `S_TIME`, …) plus:

| Command | Description |
|---|---|
| `G0 [X<mm>] [F<mm/min>]` | Absolute move / set feedrate. A bare `G0` reports the state. Same as ErisStepper |
| `EN <0\|1>` | Release / energize the motor |
| `ZERO` | Set the current position to 0 mm |
| `STOP` | Decelerate to a halt |
| `TARE` | Re-zero the load cell (unloaded) |
| `LCAL <N>` | Calibrate with a known force on the cell |
| `LSCALE [counts/N]` | Report the scale, or restore a saved one (arms the guard) |
| `LC` | Load cell + FSR status: raw, offset, scale, force, voltage, guard |
| `CAL` | Dump the buffered samples as text |

`TARE` and `LCAL` are carried out by the sampling thread over its own next
`LOADCELL_AVG_SAMPLES` conversions. Only that thread ever touches the HX711, so
a command typed mid-stream never collides with a read.

## Streaming

`S_F` accepts `FSR` and `SINE`. The board boots streaming `FSR` in ASCII mode.
The Python driver switches to binary on connect.

```
DESC FSR timestamp:f32:t,voltage:f32,force:f32,position:f32,raw:i32:m,fresh:u8:m,overload:u8:m,pad0:u8:p,pad1:u8:p
```

| Field | Meaning |
|---|---|
| `voltage` | FSR divider tap, V (12-bit ADC, `FSR_OVERSAMPLE` reads averaged) |
| `force` | N, or tared counts while uncalibrated |
| `position` | Axis position, mm |
| `raw` | HX711 counts with no tare and no scale applied. Lets you re-calibrate a recording offline |
| `fresh` | 1 = real load-cell reading. 0 = the HX711 is absent and this is an FSR-only sample (force 0) |
| `overload` | 1 = the guard is holding the axis |

```python
e = Eris(['FSR'], port='COM3')     # format comes from DESC
```

**Rate:** one sample per HX711 conversion (10 or 80 Hz). The thread polls at
`CAL_POLL_HZ` (200 Hz) and emits a row only when the HX711 has a new result, so
force and voltage are at most 5 ms apart. If no HX711 is connected, the FSR
still streams at `CAL_FALLBACK_HZ` with `fresh=0`, so you can check the divider
on its own.

## Files

| File | Contents |
|---|---|
| `motion.cpp` | A4988 STEP/DIR thread (from ErisStepper, TMC2209 parts removed) plus the press guard |
| `gcode.cpp` | `G0` parser, **identical to ErisStepper's** (check_drift will show it if they diverge) |
| `loadcell.cpp` | HX711 + FSR sampling (`ERIS_SENSOR_EX`), tare/calibration, overload guard |
| `customtypes.h` | `FSRCalSample_t` + descriptor |

## Dependencies

eriscommon, ChRt or Seeed_Arduino_FreeRTOS, SerialCommand, PacketSerial, and
**HX711 Arduino Library** (bogde, 0.7.x). The `.ino` includes `<HX711.h>`
literally so the Arduino builder finds it.

## Notes

- The load-cell scale is not stored in EEPROM, because that isn't portable across Eris boards. Paste it into `configuration.h`, or send `LSCALE <value>` after each boot.
- Default geometry: 200 steps/rev, 8 mm lead, 16 microsteps, giving 400 steps/mm. Change `STEPPER_MM_PER_REV` for your screw.
- Feeds are slow by design (60 mm/min default, 600 max). Let the FSR settle at each point. FSRs creep, so hold time matters for repeatable data.
