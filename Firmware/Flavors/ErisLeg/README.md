# ErisLeg

Lower-limb exoskeleton firmware. Drives knee and ankle joints over CAN (via Elmo Gold drives), reads a 6-axis SRI loadcell on the foot, plus FSR + sync.

## Status
Experimental (per `Flavors/README.txt`: "Not to expect updates until new device assembly")

## Hardware
- **Target board:** Teensy 3.x/4.x with FlexCAN
- **RTOS:** Auto-detected ChibiOS or FreeRTOS
- **Sensor / interface:**
  - 2x Elmo Gold servo drives over CAN (knee + ankle)
  - SRI 6-DOF loadcell over CAN (foot)
  - FSR + sync via analog/digital pins

## Pin assignments
- `PIN_LED 13`, `PIN_LED_R 28`
- `PIN_FSR A1` — `FSR_NUMCHANNELS 1` @ 1 kHz
- `PIN_SYNC 29`
- IMU pins (defined but unused in this build): `PIN_IMU_TRUNK 8`, `PIN_IMU_THIGH 7`, `PIN_IMU_SHANK 6`, `PIN_IMU_FOOT 5`
- CAN IDs:
  - `ELMO_CANID0 126` (knee), `ELMO_CANID1 127` (ankle)
  - `SRILOADCELL_CANID 128` (foot loadcell)
- Drive parameters: `ELMO_KTCONSTANT 0.038 Nm/A`, `ELMO_RATE_US 4000` (250 Hz), gear ratios `KNEE_GEAR_RATIO 120`, `ANKLE_GEAR_RATIO 175`

## Dependencies
- eriscommon
- ChRt or Arduino_FreeRTOS
- SerialCommand, PacketSerial
- SPI
- elmo (Elmo Gold CAN driver)
- sriloadcell (SRI 6-DOF CAN loadcell driver)
- FlexCAN

## Serial Commands
| Command | Description |
|---------|-------------|
| `INFO` | Print firmware info |
| `ON` / `OFF` | LED on/off |
| `SINE` / `FSR` / `SYNC` | Dump named buffer |
| `JOINT` | Dump knee + ankle joint state (theta, theta_dot, torque) |
| `LC` | Dump loadcell forces and moments |
| `IP <kK> <bK> <eqK> <kA> <bA> <eqA>` | Set impedance params for both joints |
| `IPK <k> <b> <theta_eq>` | Set knee impedance |
| `IPA <k> <b> <theta_eq>` | Set ankle impedance |
| `IP?` | Query current impedance settings |
| `S_F <feat...>` / `S_ON` / `S_OFF` | Streaming control |
| `START` | Restart Joints + Loadcell threads |
| `KILL` | Stop Joints + Loadcell threads |

## Streaming Features
`S_F` accepts: `KNEE`, `ANKLE`, `LC`, `SINE`, `FSR`, `SYNC`.

## Sample Session
```
> INFO
> IPK 50 0.5 0
> IPA 30 0.3 -5
> S_F KNEE ANKLE LC
> S_ON
... joint state + 6-DOF loadcell stream ...
> S_OFF
> KILL
```

## Notes
- `KILL`/`START` actually stop and restart the joint and loadcell drivers (unlike most flavors where they are stubs).
- This is the only flavor with hardware impedance control commands (`IP`, `IPK`, `IPA`, `IP?`).
- Filename is `ErisLeg3.0.ino` (not `ErisLeg.ino`).
