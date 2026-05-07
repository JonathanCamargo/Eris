# Eris Serial Commands Reference

Eris flavors expose ASCII commands over the USB CDC serial port. Send each command on its own line (newline-terminated). Responses come back either as plain `Serial.print` text or as COBS-framed packets when streaming is active (see the [protocol section in the main README](../README.md#serial-protocol)).

This document is the union of commands across all active flavors. Not every flavor implements every command — see the per-flavor `README.md` for what each one accepts.

## Core commands (present in every flavor)

| Command | Purpose | Example |
|---------|---------|---------|
| `INFO` | Print firmware info string. | `INFO` |
| `S_F <feat1> <feat2> ...` | Set streaming features (must match names registered in flavor's `streaming.cpp`). | `S_F SINE FSR` |
| `S_ON` | Start the streaming thread. | `S_ON` |
| `S_OFF` | Stop streaming. | `S_OFF` |
| `S_TIME` | Reset `t0` so subsequent timestamps are zero-based. Some flavors use `TIME0` instead. | `S_TIME` |
| `ON` / `OFF` | LED on/off. | `ON` |
| `START` / `KILL` | Start / stop sensor threads. *Experimental, not implemented in all flavors.* | `START` |
| `HELLO [name]` | Echo (used as a liveness check; only some flavors). | `HELLO world` |

## Streaming features

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
