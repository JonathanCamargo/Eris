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
- `IMU_COUNT 2`, `IMU_FREQUENCY_HZ 250` (4 ms tick). The rate must divide 1000 evenly — `ERIS_SENSOR_MULTI` static_asserts it.
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
| `S_MODE ASCII [TIME]` | Stream plain text (add `TIME` to include timestamps) |
| `S_MODE BIN` | Stream the COBS binary protocol (what the Python driver uses) |
| `S_MODE` | Report the current mode |
| `DESC` | Firmware describes its own wire format for the selected features |
| `START` / `KILL` | Start/kill threads (stubs) |

## Streaming Features
`S_F` accepts: `SINE`, `IMU_0`, `IMU_1`.

## Quick start (no host script needed)

Flash the board, open the Arduino **Serial Monitor** at 115200. Both IMUs stream
as text immediately — nothing to type:

```
IMU_0.ax:0.123,IMU_0.ay:-0.045,IMU_0.az:9.791,IMU_0.wx:0.001,...,IMU_1.ax:0.02,...
```

Open the **Serial Plotter** instead and every signal is graphed and labelled,
because the `label:value` form is what the Plotter parses. The `IMU_0.` /
`IMU_1.` prefixes keep the two devices apart in the legend.

This is configured in `ErisMPU.ino`:

```c
SerialCom::bootDefaults("IMU_0 IMU_1", true);   // true = ASCII
```

Change the feature list or pass `false` to boot into the binary protocol.

> **ASCII mode is a monitor, not a log.** It prints only the newest sample per
> feature, at 20 Hz, so the output stays readable and the Plotter keeps up. The
> other samples are drained from the buffer but not printed. For a complete
> record use binary mode and the Python driver.

## Sample Session (binary / Python driver)
```
> S_MODE BIN
> INFO
> INITIMU
> S_F IMU_0 IMU_1
> TIME0
> S_ON
... COBS-framed binary at the full 250 Hz ...
> S_OFF
```
The Python driver sends `S_MODE BIN` automatically on connect.

## Self-describing wire format

`DESC` makes the firmware report the layout of whatever `S_F` selected:

```
DESC IMU_0 timestamp:f32:t,ax:f32,ay:f32,az:f32,wx:f32,wy:f32,wz:f32
DESC IMU_1 timestamp:f32:t,ax:f32,ay:f32,az:f32,wx:f32,wy:f32,wz:f32
DESC END
```

Role suffix: `:t` time, `:m` meta, `:p` padding; absent for a plain signal.
Unlike ASCII output this lists *every* field — it describes the binary layout.

The description is emitted through the same `StreamSamples()` calls, walking the
same feature list in the same order, as the data path. So the reported schema
cannot drift from the bytes actually sent.

This is what lets the Python driver configure itself:

```python
e = Eris(['IMU_0','IMU_1'], port='COM3')   # no format= needed
```

Pass `format=` explicitly to override, or for firmware without `DESC`.

## How the acquisition is written

This flavor is the reference for `ERIS_SENSOR_MULTI`. The whole polling loop is
one declaration in `imu.cpp`:

```c
ERIS_SENSOR_MULTI(IMU, IMUSample_t, IMU_FREQUENCY_HZ, IMU_COUNT) {
  if (!ok[ch]) return false;          // absent device: append nothing
  dev[ch]->readSensor();
  s.ax = dev[ch]->getAccelX_mss();
  ...
  return true;
}
```

That generates `IMU::buffer[2]`, `IMU::start()`, and a 250 Hz thread that
timestamps each sample and appends it. You write the device reads and nothing
else — no working area, no stack tier, no thread priority, no timing loop.

Two devices are read by **one** thread on purpose: devices sharing an I2C bus
need a single owner, and all channels in a tick share one timestamp because
they are one tick.

`IMU::begin()` (called from the `.ino`) brings up `Wire` and the chips, then
calls the generated `start()`.

`IMUSample_t` carries a field descriptor in `customtypes.h`. If you add a field
to the struct and forget the descriptor, the build fails — that is deliberate,
because a silently missing column is very hard to notice in recorded data.

Full walkthrough for a new sensor: [`doc/ADDING_A_SENSOR.md`](../../../doc/ADDING_A_SENSOR.md).

### When *not* to use ERIS_SENSOR

`ERIS_SENSOR` / `ERIS_SENSOR_MULTI` are for fixed-rate polling only. If your
sensor is interrupt-driven (a DRDY line), DMA/burst driven, multi-rate, or
gated on anything other than time, **do not bend the macro to fit** — write the
thread by hand with `ERIS_THREAD_WA` / `ERIS_THREAD_FUNC` / `eris_thread_create`
and append to your own `ErisBuffer` from wherever the data arrives. Every other
Eris tool still applies: buffers, streaming templates, the packet layer, the
error handler. Only the polling loop is missing, because your data is not
polled. `ErisADS1299` is the worked interrupt-driven example.

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
- Timestamps are hidden in ASCII mode by default — a monotonically rising trace
  wrecks the Serial Plotter's autoscale. `S_MODE ASCII TIME` includes them.
- ASCII output only covers sample types that have an `ERIS_DESCRIBE` block. An
  undescribed type prints nothing; binary mode is unaffected.
