# ErisMPU

6-DOF IMU streaming firmware. Reads up to two MPU9250 accelerometer/gyroscope chips over I2C at 250 Hz.

## Status
Verified working (per project memory)

## Hardware
- **Target board:** SAMD21 (Seeeduino XIAO) — uses `TimerTC3` for the sample tick. Compatible with any board that has a TimerTC3 port.
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:** MPU9250 over I2C @ `0x68` (imu0, enabled by default) and `0x69` (imu1, disabled by default)

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- I2C: `Wire` (default SDA/SCL for the target board)
- `IMU_FREQUENCY_HZ 250` (4 ms tick driven by TimerTC3)
- `IMU_TXBUFFERSIZE 10`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- Wire (I2C, Arduino built-in)
- MPU9250 (Brian Taylor's library)
- TimerTC3 (SAMD21 timer)

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
> S_F IMU_0
> TIME0
> S_ON
... ax/ay/az + wx/wy/wz samples stream at 250 Hz ...
> S_OFF
```

## Notes
- A second IMU on `0x69` can be enabled by setting `imu1OK=true` in `imu.cpp`.
- The IMU sample is captured in the timer ISR, so reads happen with bounded jitter.
- This flavor uses `TIME0` (legacy) instead of `S_TIME`.
