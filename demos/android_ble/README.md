# Eris over Bluetooth LE: Android demo

A phone app that talks to an Eris board over Bluetooth LE. It shows:

- **Live plots** of streamed data, decoded from the firmware's own description of
  its wire format (`DESC`), so the app has no hard-coded struct layouts.
- **Command buttons**, each sending one Eris command line, the same as typing it in
  the Arduino Serial Monitor.
- **Stream toggles.** Tick or untick streams and the app reconfigures the board
  (`S_OFF` → `S_F …` → `DESC` → `S_ON`).

It is a Robiolab app: Expo / React Native with the shared `@robiolab/native-ui` package, so it
has the same look, icon and release process as `robiolab_inventory_app`.

```
demos/android_ble/
  firmware/ErisBLEDemo/   Arduino sketch for a Seeed XIAO nRF52840
  app/                    Expo (React Native) app
    app/                  screens (expo-router)
    src/protocol/         COBS, DESC schema, packet session  (no React Native imports)
    src/link/             BLE (react-native-ble-plx) and the built-in simulator
    src/store/            app state + the stream-configuration sequence (zustand)
    src/plot/             trace buffers and the live chart (Skia)
    src/screens/          Connect and Dashboard
    src/demoCommands.ts   the command buttons and default streams
    tests/                jest tests for the protocol layer
    scripts/              release APK build, same as the other Robiolab apps
```

## 1. Flash the firmware

Needs the **Seeed nRF52** board package (board: *Seeed XIAO nRF52840*), plus the
`eriscommon`, `Arduino-SerialCommand` and `PacketSerial` libraries from
ArduinoLibraries installed in your sketchbook.

Open `firmware/ErisBLEDemo/ErisBLEDemo.ino` and upload it. From the command line:

```
arduino-cli compile -b Seeeduino:nrf52:xiaonRF52840 demos/android_ble/firmware/ErisBLEDemo
arduino-cli upload  -b Seeeduino:nrf52:xiaonRF52840 -p COMx demos/android_ble/firmware/ErisBLEDemo
```

The board advertises as **Eris-Demo** (`ERIS_BLE_NAME` in `configuration.h`).
It needs no USB host, so run it from a battery or a charger.

Streams it offers (none need wiring):

| Stream   | Rate   | Fields                        | What it is |
|----------|--------|-------------------------------|------------|
| `WAVES`  | 100 Hz | `sine`, `triangle`, `square`  | Generator; change it with `FREQ` / `AMP` |
| `ANALOG` | 50 Hz  | `a0`, `a1` (volts)            | Pins A0, A1. Touch them or wire a potentiometer |
| `TEMP`   | 10 Hz  | `value` (°C)                  | nRF52 die temperature |
| `SINE`   | 100 Hz | `value`                       | eriscommon's reference sine (very slow: 100 s period) |

Demo commands (in addition to the universal `INFO`, `S_TIME`, `S_F`, `S_ON`, …):

| Command | Reply | Effect |
|---------|-------|--------|
| `FREQ <hz>` | `FREQ 2.00 Hz` | WAVES frequency, 0.1–20 Hz |
| `AMP <a>`   | `AMP 3.00`     | WAVES amplitude, 0–10 |
| `RGB <R\|G\|B\|W\|OFF>` | `RGB G` | On-board RGB LED |
| `PING`      | `PONG 12345 ms` | Liveness check |
| `STATUS`    | `STATUS freq=… amp=… rgb=… up=…s` | Current settings |

## 2. Build and install the app

Prerequisites: Node 20+, Android Studio (SDK 36, NDK 27.1), and **`robiolab_native_ui` checked out
next to the Eris repo** (`C:\git\robiolab\robiolab_native_ui` beside `C:\git\Eris`), because the
app depends on it by path until that package has its own git repository. npm installs a
copy of it, which a plain `npm install` never refreshes: after changing `robiolab_native_ui`,
bump its version and run `npm run ui:update`.

Release APK, exactly as for the other Robiolab apps:

```powershell
cd demos/android_ble/app
.\scripts\build-release-apk.ps1        # or: bash scripts/build-release-apk.sh
```

Output: `android/app/build/outputs/apk/release/app-release.apk`. Install it with
`adb install -r <apk>`.

For development on a phone connected over USB debugging: `npm install`, then
`npx expo run:android`. **Expo Go does not work**: BLE needs the native
`react-native-ble-plx` module, which only a development or release build contains.

On Android 12+ the app asks for *Nearby devices*. On Android 8–11 it asks for location,
and **Location must be switched on** in quick settings, or scans return nothing (an
Android rule for BLE).

## 3. Use it

1. **Scan for boards** lists devices advertising the Nordic UART Service. Tap yours.
2. The app puts the board in a known state (`S_OFF`, `S_MODE BIN`, `INFO`), then starts
   `WAVES`.
3. Tick streams to add charts. Tap a legend entry to hide a series. Long-press a chart
   and drag to freeze it and read exact values.
4. Press the command buttons, or type any command. Replies appear in the Console.

**No board?** Tap **Try the simulator**. It emulates ErisBLEDemo byte-for-byte: COBS
frames split into MTU-sized notifications, DESC replies, and error packets. Everything
past the radio runs the same code, and it works on an emulator.

## How it talks to Eris

Eris's BLE transport (`eriscommon/src/modules/ble_transport.*`) exposes the **Nordic
UART Service** as an Arduino `Stream`, so the ordinary Eris protocol runs over it
unchanged:

| Direction | Characteristic | Content |
|-----------|----------------|---------|
| phone → board | `6e400002-…` (RX, write) | Command lines, `\n`-terminated |
| board → phone | `6e400003-…` (TX, notify) | COBS-encoded packets, each ended by `0x00` |

Notifications don't line up with packets, so the app appends every notification to a
buffer and cuts frames at `0x00` (`src/protocol/cobs.ts`, `FrameSplitter`). The first
byte of each decoded packet is its type: `D` data, `T` text, `E` error.

Changing streams always runs the same sequence (`applyStreams` in `src/store/erisStore.ts`):

```
S_OFF                        stop, so no packet of the old layout is in flight
S_F WAVES TEMP               select streams
DESC                         board replies, one TEXT packet per stream:
  DESC WAVES timestamp:f32:t,sine:f32,triangle:f32,square:f32
  DESC TEMP timestamp:f32:t,value:f32
  DESC END
S_ON                         data resumes
```

A `D` packet is then, per stream in `S_F` order, a `uint8` sample count followed by that
many little-endian structs, laid out as `DESC` described (`decodeData` in
`src/protocol/schema.ts`). A packet that doesn't fit the layout exactly is dropped and
counted as "skipped" rather than misread.

If you tick a stream the firmware doesn't have, `S_F` rejects the whole set. The app then
probes each name on its own, unticks the unknown one, and says why.

## Adapting it to your own flavor

Any Eris flavor works if it enables BLE the same way ErisBLEDemo does:

1. In `configuration.h`: `#define ERIS_USE_BLE` and `#define ERIS_BLE_NAME "…"`.
2. In the `.ino`: `#include <bluefruit.h>` under `#ifdef ERIS_USE_BLE`. This line is
   **required**; without it the BLE code silently compiles to nothing. Then call
   `SerialCom::useBLE(ERIS_BLE_NAME)` in `setup()`, before `ERIS_RUN`.
3. Give every streamed sample type an `ERIS_DESCRIBE` block (`eris_descriptor.h`), so
   `DESC` can report it.
4. Pass the feature label to `StreamSamples(…, "NAME")` so DESC reports the right name.

In the app: add stream names with **+ Add** (or edit `DEFAULT_STREAMS`), and add buttons
in `src/demoCommands.ts`.

To add a stream to the demo firmware: write the sensor with `ERIS_SENSOR` in
`signals.cpp`, add a line to the table in `streaming.cpp`, and it becomes tickable from
the phone.

## Branding

Colors, radius, shadow, components (Button, Card, Chip, SegmentedControl, TextField,
Banner, …), the launcher icon and the splash all come from `@robiolab/native-ui`; this app
declares none of its own. Chart series use the package's `chart` palette: the logo's
teal, coral and bronze, checked for color-blind separation and contrast on white. It has
three slots, so a stream with more signals (e.g. an IMU's six) is drawn as small multiples
of up to three.

## Limits and gotchas

- **Only packets reach the phone.** Replies that eriscommon prints with `Serial.println`
  go to the unplugged USB port instead: `Features Ready`, `Mode: BIN`, `LED on`. That's
  why the demo's own commands reply with TEXT packets, and why the app confirms `S_F`
  through `DESC` rather than waiting for text.
- **Binary mode only.** The app always switches the board to `S_MODE BIN`.
- **Throughput.** On BLE the streaming thread runs every 100 ms
  (`ERIS_BLE_STREAM_PERIOD_MS`), batching up to 16 samples per stream per packet. All four
  demo streams together are about 3 kB/s.
- **C++11 on nRF52.** The Seeed nRF52 core compiles with `-std=gnu++11`. The generated
  descriptors for `AnalogSample<N>` and `ERIS_DESCRIBE_CHANNELS` need C++14 and fail to
  compile there. Describe nRF52 sample types field by field, as `customtypes.h` does.
- **RGB polarity.** `RGB_LED_ON` assumes the XIAO's LED lights on `LOW`. Swap the two
  defines if the colors come out inverted.

## Tests

```
cd demos/android_ble/app
npm test
```

jest tests for COBS, frame reassembly at every chunk size, base64, DESC parsing, D-packet
decoding, the session, and the trace buffer. The COBS and data-packet vectors were
generated with the Python `cobs` package, the same one the Eris Python driver uses:

```python
import struct; from cobs import cobs
body = b'D' + struct.pack('<B4f4f', 2, 10, .5, -.25, 1, 20, 0, 1, -1) + struct.pack('<B2f', 1, 15, 3.3)
cobs.encode(body)
```
