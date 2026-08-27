# ErisMPU

6-DOF IMU streaming firmware. Reads two MPU9250 accelerometer/gyroscope chips over I2C at 250 Hz.

## Status
Verified working (per project memory)

## Hardware
- **Target board:** board-agnostic. Developed on SAMD21 (Seeeduino XIAO); the sample tick is an RTOS thread (`eris_sleep_until`), so any board `eris_rtos.h` supports will run it (Teensy, SAMD21, nRF52, ESP32).
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** MPU9250 over I2C @ `0x68` (imu0) and `0x69` (imu1) — both initialized by default; `imu1` needs `AD0` pulled HIGH

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- I2C: `Wire` (default SDA/SCL for the target board)
- `IMU_FREQUENCY_HZ 250` (4 ms tick driven by the IMU sampling thread; keep it a divisor of 1000 so the period is a whole number of scheduler ticks)
- `IMU_TXBUFFERSIZE 10`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- Wire (I2C, Arduino built-in)
- MPU9250 (Brian Taylor's library)

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` | Dump SineWave buffer |
| `INITIMU` | Re-initialize the IMU(s) |
| `IMU [0|1]` | Dump buffer for IMU index 0 (default) or 1 |
| `FAIL` | Print initialization-failure code |
| `TIME0` | Reset `t0` |
| `S_F <feat...>` | Set streaming features |
| `S_ON` / `S_OFF` | Start/stop streaming |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `IMU_0`, `IMU_1`.

## Sample Session
```
> INFO
> INITIMU
> S_F IMU_0 IMU_1
> TIME0
> S_ON
... ax/ay/az + wx/wy/wz samples stream at 250 Hz ...
> S_OFF
```

## Notes
- **Both IMUs are initialized by default.** `imu0` @ `0x68`, `imu1` @ `0x69`.
  `imu1` only answers at `0x69` if its `AD0`/`SDO` pin is pulled **HIGH** — left
  floating or tied LOW it collides with `imu0` and neither enumerates reliably.
- An absent IMU is not an error: `begin()` fails, its `imuXOK` flag stays false,
  and the sampling thread skips it. `FAIL` reports which ones failed
  (bit 3 = IMU 0, bit 2 = IMU 1; `0` = both up, `-1` = `INITIMU` never ran).
- Accel calibration is identity for both. Those constants are **per physical
  chip** — measure them per unit, never copy between devices.
- Both IMUs are read back-to-back in the same 4 ms tick and share a timestamp.
  At the driver's 400 kHz I2C clock two reads cost roughly 400 us of the 4 ms
  budget.
- This flavor uses `TIME0` (legacy) instead of `S_TIME`.
