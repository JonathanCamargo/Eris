# Eris Serial Commands Reference

Eris flavors expose ASCII commands over the USB CDC serial port. Send each command on its own line (newline-terminated). Responses come back either as plain `Serial.print` text or as COBS-framed packets when streaming is active (see the [protocol section in the main README](../README.md#serial-protocol)).

Adding a sensor of your own? See [ADDING_A_SENSOR.md](ADDING_A_SENSOR.md).

This document is the union of commands across all active flavors. Not every flavor implements every command — see the per-flavor `README.md` for what each one accepts.

## Core commands (present in every flavor)

| Command | Purpose | Example |
|---------|---------|---------|
| `INFO` | Print firmware info string. | `INFO` |
| `S_F <feat1> <feat2> ...` | Set streaming features (must match names registered in flavor's `streaming.cpp`). | `S_F SINE FSR` |
| `S_ON` | Start the streaming thread. | `S_ON` |
| `S_OFF` | Stop streaming. | `S_OFF` |
| `S_MODE ASCII [TIME]` | Stream plain text (`label:value` pairs) instead of binary. The Arduino Serial Plotter graphs it directly. Add `TIME` to include timestamps (off by default: a rising trace wrecks the Plotter's autoscale). | `S_MODE ASCII` |
| `S_MODE BIN` | Stream the COBS binary protocol. What the Python driver uses; the default unless a flavor calls `SerialCom::bootDefaults(..., true)`. | `S_MODE BIN` |
| `S_MODE` | Report the current mode. | `S_MODE` |
| `DESC` | Firmware describes its own wire format for the features `S_F` selected. One TEXT packet per feature, then `DESC END`. | `DESC` |
| `S_TIME` | Reset `t0` so subsequent timestamps are zero-based. Some flavors use `TIME0` instead. | `S_TIME` |
| `ON` / `OFF` | LED on/off. | `ON` |
| `START` / `KILL` | Start / stop sensor threads. *Experimental, not implemented in all flavors.* | `START` |
| `HELLO [name]` | Echo (used as a liveness check; only some flavors). | `HELLO world` |

## Streaming features

## Self-describing wire format (`DESC`)

`DESC` reports the binary layout of the currently selected features:

```
DESC IMU_0 timestamp:f32:t,ax:f32,ay:f32,az:f32,wx:f32,wy:f32,wz:f32
DESC END
```

Each field is `name:type[:role]` — types `f32 u8 i8 u16 i16 u32 i32 pad`, roles
`:t` time, `:m` meta, `:p` padding (absent = plain signal). The list covers every
byte of the struct, so a host can build a decoder from it directly:

```python
e = Eris(['IMU_0','IMU_1'], port='COM3')   # layout fetched via DESC
```

The schema is emitted through the same `StreamSamples()` calls, in the same
order, as the data itself — so it cannot drift from the bytes on the wire.

A feature whose sample type has no `ERIS_DESCRIBE` block reports
`UNDESCRIBED`; the host then needs an explicit `format=`. See
`eriscommon/src/eris_descriptor.h`.

## ASCII mode

`S_MODE ASCII` renders samples as text lines:

```
IMU_0.ax:0.123,IMU_0.ay:-0.045,IMU_0.az:9.791,IMU_1.ax:0.020
```

**It is a monitor, not a log**: only the newest sample per feature is printed,
at 20 Hz, so Serial Monitor stays readable and the Plotter keeps up. Use binary
mode for a complete record.

The `S_F` command activates one or more features whose names are matched against the `strncmp(...)` ladder in each flavor's `streaming.cpp`. Common features:

| Feature | Sample type | Description |
|---------|-------------|-------------|
| `SINE` | `floatSample_t` | Built-in 1 Hz reference sine; available in nearly every flavor |
| `FSR` | `FSRSample_t` | Force-sensitive resistors (analog channels) |
| `EMG` | `EMGSample_t` | EMG samples (ADS1299, ADS131, or analog depending on flavor) |
| `IMU` | `IMUSample_t` | 6-DOF accelerometer + gyroscope (MPU9250 or similar) |
| `SYNC` | `uint8_tSample_t` | Digital sync-input edge timestamps |
| `BIOM` / `FEAT` / `CLASS` / `REG` | flavor-specific | Biomechanics, feature, classifier, regression outputs |

## SD-card recording (where available)

Flavors that include `eris_sd.h` and a `sdcard.cpp` add:

| Command | Purpose |
|---------|---------|
| `SD_REC <name>` | Start recording with trial name `<name>`. Files are written under `<name>/` per buffer (e.g. `<name>/fsr.bin`). |
| `SD_NREC` | Stop recording. |

## Servo commands (`ErisServo`, `ErisServoHand`, `ErisServoINA`)

| Command | Purpose | Example |
|---------|---------|---------|
| `X <ch> <angle>` | Immediate move of one servo. | `X 0 90` |
| `X <a0> ... <aN>` | Immediate move of all servos. | `X 0 30 60 90 120 150` |
| `Y <ch> <angle>` | Smooth (rate-limited) move of one servo. | `Y 0 45` |
| `Y <a0> ... <aN>` | Smooth move of all servos. | `Y 0 30 60 90 120 150` |
| `OPEN` / `CLOSE` / `SHAKA` | (ServoHand) preset poses. | `OPEN` |
| `INA` | (ServoINA) print INA219 current readings. | `INA` |

## ADS1299 / EMG commands (`ErisADS1299`, `ErisADS131`, `ErisAnalog`)

| Command | Purpose |
|---------|---------|
| `EMG` | Print latest EMG buffer contents. |
| `A` | Print latest analog samples (ErisAnalog). |
| `EMG2` | Alternate channel set (selected flavors). |

## Motor / actuator commands (`ErisDCMotor`, `ErisLeg`)

| Command | Purpose |
|---------|---------|
| `MOT_F` / `MOT_B` | Run motor forward / backward. |
| `MOT_STOP` / `MOT_OFF` | Stop / disable the motor. |
| `POT` | Print current potentiometer value. |
| `LC` | Print latest load-cell sample (`ErisLeg`). |
| `JOINT` | Print joint angles (`ErisLeg`). |
| `IPK` / `IPA` / `IP?` / `IP` | PID-tuning helpers (gains, query). |

## Stepper / G-code commands (`ErisStepper`)

A TMC2209 axis: STEP/DIR for motion, UART for configuration. Coordinates are
absolute millimetres; `F` is millimetres per **minute**, the G-code convention,
and persists until changed. Words are space-separated (`G0 X10 F600`) because
Eris tokenizes on whitespace — `G0X10` will not parse. `G1` is deliberately not
implemented; see the flavor README.

| Command | Purpose |
|---------|---------|
| `G0 X<mm>` | Move to an absolute position at the current feedrate. |
| `G0 X<mm> F<mm/min>` | Set the feedrate, then move at it. |
| `G0 F<mm/min>` | Set the feedrate only; nothing moves. |
| `G0` | Report position, target, feedrate, velocity and moving/idle. |
| `STEP` | Print the buffered axis telemetry as text. |
| `TMC` | Driver status: UART link, chip version, decoded `DRV_STATUS`. |
| `TMC I <mA>` | Set motor RMS current over UART, live. |
| `TMC U <n>` | Set microsteps (power of two, 1..256) over UART, live. |
| `TMC S <0\|1>` | SpreadCycle / StealthChop over UART, live. |
| `SG` | Report the live StallGuard4 load reading, its threshold, and whether the hard-stop detector is armed / tripped. |
| `SG <0..255>` | Set the stall threshold (`SGTHRS`) live. Higher trips earlier; must be tuned per machine. |
| `SG CLR` | Clear the latched stall flag (a new `G0`, `ZERO` or `EN 1` also clears it). |
| `EN <0\|1>` | Release / energize the motor (releasing also cancels the move). |
| `ZERO` | Define the current position as 0 mm. |
| `STOP` | Decelerate to a halt and cancel the move. |

Hard-stop detection is StallGuard4-based, not current-based: a chopper driver
holds coil current constant, so hitting a stop does not raise it. See the flavor
README for how to tune `SGTHRS`.

## NextFlex commands

| Command | Purpose |
|---------|---------|
| `ETI` | Toggle/query ETI driver mode. |
| `TPAC` | TPAC sensor read (`ErisTapok`). |
| `GAIT` | Stream gait-phase output (`ErisTapok2`). |

## Feature-stream control (where present)

| Command | Purpose |
|---------|---------|
| `F_ON` / `F_OFF` | Enable / disable on-device feature extraction. |
| `F_INFO` | Report current feature config. |
| `F_R <n>` / `F_WIN <ms>` | Set feature decimation / window. |
| `F_MASK <bits>` | Mask which features are streamed. |
| `F_IDX` / `F_CLASS` | Report classifier index / class label. |
| `FAIL` | Inject a fail packet (testing). |

## How to discover what a flavor supports

```bash
grep '"' Firmware/Flavors/<Flavor>/serialcommand.cpp | grep addCommand
grep '"' Firmware/Flavors/<Flavor>/streaming.cpp     | grep strncmp
```

That dump is authoritative; this document is a curated overview.
