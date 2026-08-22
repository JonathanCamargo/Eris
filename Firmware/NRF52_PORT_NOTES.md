# nRF52 Port Notes (Seeed XIAO nRF52840)

## Issues Encountered & Fixes Applied

### 1. `SerialCommand` library not linked for nRF52

**Problem:** The `Arduino-SerialCommand` library had no `library.properties` file, only a `library.json` declaring `"platforms": "atmelavr"`. The Arduino IDE excluded it for the nRF52 architecture.

**Fix:** Created `library.properties` with `architectures=*` in the library folder.

### 2. Windows case-insensitive filename collision

**Problem:** The local flavor file `serialcommand.h` (all lowercase) collided with the library header `SerialCommand.h` on Windows' case-insensitive filesystem. When the compiler resolved `<SerialCommand.h>`, it found the local `serialcommand.h` instead, causing `'SerialCommand' does not name a type` errors.

**Fix:** Renamed flavor-specific files from `serialcommand.h`/`.cpp` to `servo_commands.h`/`.cpp` (in ErisServo). Other flavors like BareMinimal already used `serialcommands.h` (with an 's').

### 3. ChRt `SVC_Handler` conflicts with nRF52 FreeRTOS

**Problem:** The nRF52 board core always links its built-in FreeRTOS, which defines `SVC_Handler`. ChRt also defines `SVC_Handler` in `chcore_v7m.c`, causing a multiple-definition linker error.

**Fix:** Made ChRt's `SVC_Handler` a weak symbol (`__attribute__((weak))`) in `chcore_v7m.c` so FreeRTOS's strong definition wins.

### 4. `eris_rtos.h` auto-detected ChRt instead of FreeRTOS on nRF52

**Problem:** `eris_rtos.h` checked `__has_include(<ChRt.h>)` first. Since the Arduino IDE always links ChRt (it sees the `#include` in sketch code), the header was available and ChibiOS was selected — but the linker was actually using FreeRTOS from the nRF52 core. This caused undefined references to ChibiOS symbols (`chThdSleep`, `chMBPostI`, etc.).

**Fix:** Added NRF52 platform macro checks (`NRF5`, `NRF52840_XXAA`, `NRF52`) before the `__has_include` checks in `eris_rtos.h`. On nRF52 boards, `ERIS_USE_FREERTOS` is now forced regardless of ChRt availability.

### 5. `vApplicationGetIdleTaskMemory` / `vApplicationGetTimerTaskMemory` duplicate

**Problem:** The nRF52 core's `rtos.cpp` already provides these FreeRTOS static-allocation callbacks. The sketch also defined them under `#ifdef ERIS_USE_FREERTOS`, causing multiple-definition errors.

**Fix:** Guarded the sketch's definitions with `!defined(NRF52_SERIES)` so they're skipped on nRF52.

### 6. FreeRTOS scheduler already running on nRF52

**Problem:** The nRF52 core starts the FreeRTOS scheduler in `main()` before `setup()` is called. Calling `vTaskStartScheduler()` again is a no-op, but the real issue was `eris_scheduler_start` calling `vTaskStartScheduler()` unnecessarily, and more importantly, `while(true){}` after `setup()` blocked the loop task forever, starving equal-priority Eris threads (nRF52 uses `configUSE_TIME_SLICING=0`).

**Fix in `eris_rtos.h`:** On nRF52, `eris_scheduler_start(fn)` just calls `fn()` — no `vTaskStartScheduler()`.

**Fix in flavors' `.ino`:** The `while(true){}` block after `eris_scheduler_start` is skipped on FreeRTOS (`#ifdef ERIS_USE_FREERTOS`). `setup()` returns, and `loop()` calls `eris_sleep_ms()` which yields properly.

### 7. `while(true){}` blocking after `setup()` (RESOLVED)

**Status:** Fixed. `ERIS_RUN(start)` replaced the old `eris_scheduler_start` + `while(true){}` idiom so the loop task no longer busy-blocks, and the equal-priority scheduling stall was cured by creating the command/stream threads *above* the loop task: `ReadSerial_T` at `ERIS_NORMAL_PRIORITY+2`, `StreamSerial_T` at `+3` (`serialcom.cpp`). Serial/BLE commands are now processed normally — confirmed end-to-end over BLE (`S_F SINE` / `S_ON` drive a clean 100 Hz stream).

**Original cause (kept for reference):** With `configUSE_TIME_SLICING=0`, equal-priority tasks don't pre-empt each other, so a ReadSerial thread created at the same priority as the loop task never got scheduled. Raising its priority above the loop task fixed it.

## Files Modified

| File | Change |
|------|--------|
| `Arduino-SerialCommand/library.properties` | Created with `architectures=*` |
| `ChRt/src/arm/chcore_v7m.c` | `SVC_Handler` marked `__attribute__((weak))` |
| `eriscommon/src/eris_rtos.h` | NRF52 platform check before ChRt auto-detection; nRF52 scheduler skip |
| `eriscommon/src/modules/serialcom.cpp` | (no changes yet — may need priority adjustment) |
| `ErisServo/Eris.h` | Conditional `#include <ChRt.h>` |
| `ErisServo/configuration.h` | NRF52 FreeRTOS detection |
| `ErisServo/ErisServo.ino` | FreeRTOS scheduler guard; renamed include |
| `ErisServo/servo_commands.h/cpp` | Renamed from `serialcommand.h/cpp` |
| `BareMinimal/Eris.h` | Conditional `#include <ChRt.h>` |
| `BareMinimal/configuration.h` | NRF52 FreeRTOS detection |
| `BareMinimal/BareMinimal.ino` | FreeRTOS scheduler guard; fixed include |

## Status

All issues above are resolved. The nRF52 (Seeed XIAO nRF52840) boots the RTOS,
responds to serial/BLE commands, and streams cleanly over both USB and BLE at
100 Hz. The BLE throughput fix is detailed under *Notes / limits → Throughput*
below and in `lessons_learned.md` §12.
## BLE transport (optional)

The nRF52 has an on-chip radio, so a flavor can stream over Bluetooth LE instead
of USB. Eris exposes the Nordic UART Service (NUS) as an Arduino `Stream` and
repoints both the outbound `Packet` and the inbound `SerialCommand` reader at it,
so the *entire* link (data out + commands in) moves to BLE. COBS framing is
unchanged; the host driver reassembles NUS notifications transparently and
resyncs on the `0x00` delimiter after any reconnect.

### Enabling (works in the Arduino IDE)

A flavor opts in at **runtime** by calling `SerialCom::useBLE(name)` from its
sketch's `setup()`, before `ERIS_RUN`. That's a normal function call in the
sketch translation unit, so it can be gated by a `configuration.h` flag — which
a `#ifdef` inside the separately-compiled `eriscommon` library never could. In
BareMinimal:

```c
// configuration.h
#define ERIS_USE_BLE                 // uncomment to enable BLE
#define ERIS_BLE_NAME "Eris-Bare"    // advertised name

// BareMinimal.ino, near the top includes -- REQUIRED (see gotcha below):
#ifdef ERIS_USE_BLE
#include <bluefruit.h>
#endif

// BareMinimal.ino, in setup() before ERIS_RUN:
#ifdef ERIS_USE_BLE
  SerialCom::useBLE(ERIS_BLE_NAME);
#endif
```

> **Build gotcha — the literal `#include <bluefruit.h>` is mandatory.** The
> eriscommon BLE module is guarded on `__has_include(<bluefruit.h>)`. Arduino
> only adds a library's include path to the build when some source *literally*
> `#include`s one of its headers, so without that one line in the sketch the
> Bluefruit path is never added, `__has_include` is always false, and the BLE
> code silently compiles to **nothing** — the firmware builds fine but never
> advertises. If `SerialCom::useBLE()` seems to do nothing, this is why.

This is the key fix vs. the original design: **don't** make the library see the
flag. The library always *offers* BLE on any board with the Bluefruit core
(`ble_transport.{h,cpp}` and the swap in `serialcom.cpp` are gated on
`__has_include(<bluefruit.h>)`); the flavor *chooses* it with the runtime call.
Build it the same way you build any nRF52 flavor — no `--build-property`, no
`-D`, just **Verify/Upload in the IDE** (or a plain `arduino-cli compile`):

```
arduino-cli compile -b Seeeduino:nrf52:xiaonRF52840 Firmware/Flavors/BareMinimal
```

Leave `ERIS_USE_BLE` commented and the build is the old USB path: `useBLE()` is
never called, the radio never starts (`bleName` stays `NULL`), and
`SerialCommand`/`Packet` keep targeting `Serial`. The BLE module still links on
nRF52, but only costs flash — no runtime effect until `useBLE()` runs.

### What it touches

| File | Change |
|------|--------|
| `eriscommon/src/modules/ble_transport.{h,cpp}` | Brings up Bluefruit NUS (MTU 247, fast conn interval) and exposes it as a back-pressured `Stream` (`BackpressureUart` — won't truncate a packet on a full radio queue). Gated on `__has_include(<bluefruit.h>)` — compiles to nothing off nRF52. |
| `eriscommon/src/packet.{h,cpp}` | New `Packet::setStream()` to repoint the data path after construction. |
| `eriscommon/src/modules/serialcom.{h,cpp}` | New `SerialCom::useBLE(name)` records the choice; `start()` does the swap (`BLETransport::start()` + repoint `packet`/`sCmd`) under `__has_include(<bluefruit.h>)` when a name was set. |
| `Arduino-SerialCommand/SerialCommand.{h,cpp}` | `setStream(Stream&)`; read loop now pulls from the configured stream (defaults to `Serial`, fully backward compatible). |
| `BareMinimal/configuration.h` | `ERIS_USE_BLE` (commented) + `ERIS_BLE_NAME` defines. |
| `BareMinimal/BareMinimal.ino` | Calls `SerialCom::useBLE(ERIS_BLE_NAME)` under `#ifdef ERIS_USE_BLE`; `while(!Serial)` boot-wait also skipped under it (no USB host when untethered). |

### Bring-up checklist (on hardware)

1. Uncomment `ERIS_USE_BLE` in `configuration.h` and flash. The board should boot
   **without** USB attached (no `while(!Serial)` hang) and start advertising as
   the configured name.
2. Connect with a generic NUS client first (nRF Connect / Bluefruit Connect app,
   UART mode). You should see COBS-framed bytes when streaming is on.
3. Send `INFO\n` over the NUS RX characteristic — expect a `T` (text) packet back.
   Then `S_F SINE\n`, `S_ON\n` to start streaming; `S_OFF\n` to stop.
4. Point the Python driver at the BLE NUS endpoint (a small `Stream`-like adapter
   over the NUS UUIDs) instead of the serial port — packet parsing is identical.

### Notes / limits

- **Throughput / dropped bytes (fixed 2026-06-30).** The original symptom was the
  host failing to COBS-decode *every* frame. Root cause: the Bluefruit `BLEUart`
  write drops bytes when the radio can't keep up, and `PacketSerial::send()` writes
  each frame in one `write()` without checking the return value — so a packet's tail
  was silently lost and the COBS length code knocked out of alignment. (The leading
  code byte still *looks* valid — e.g. `0x83` = 131 is correct for a 130 B zero-free
  `D` packet of 16 samples; the bytes were simply missing.) Three fixes in
  `ble_transport.cpp`, verified at full 100 Hz (MTU 247, 30/30 frames, 0 drops):
    1. `Bluefruit.configPrphBandwidth(BANDWIDTH_MAX)` **before** `begin()` → ATT MTU
       247, so a whole ~131 B packet rides in one notification instead of 20 B
       fragments.
    2. `Bluefruit.Periph.setConnInterval(6, 12)` → request a 7.5–15 ms connection
       interval (more notifications per second).
    3. `BackpressureUart` — a `Stream` wrapper whose `write()` loops until every byte
       is accepted (yielding between attempts), so a full radio queue back-pressures
       instead of truncating the packet. This is the actual byte-loss cure.

  Still active and complementary: `start()` sets `streamPeriodMs =
  ERIS_BLE_STREAM_PERIOD_MS` (default 100 ms) so each packet batches ~16 samples —
  fewer, fuller notifications. **NOTE:** `bufferTXD()` was tried and then *removed* —
  with the large MTU it bought nothing and would hold a packet's sub-MTU tail until
  the next packet, stalling command replies. If drops ever recur it's link byte-loss
  (raise `ERIS_BLE_STREAM_PERIOD_MS` or shrink `TXBUFFERSIZE`), not a framing bug.
  See `lessons_learned.md` §12.
- **Single transport at a time.** This is the boot-time USB↔BLE swap, not
  simultaneous dual transport (the `ErisBuffer` is single-consumer, so two
  streamers on one buffer would split the data, not duplicate it).
- **Raw `Serial.println` status lines** (e.g. "Serial Commands are ready") still
  target USB and are simply lost when untethered — harmless. Data and command
  echoes go over BLE.
- **Task priorities:** the BLE SoftDevice has hard radio timing; keep sensor
  acquisition threads from starving it. Current priorities (ReadSerial `+2`,
  Stream `+3`) leave the SoftDevice's high-priority IRQs untouched.
