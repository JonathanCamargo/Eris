# Eris — Lessons Learned

Practical, hard-won knowledge from porting and extending Eris flavors. Each entry
is a real problem that cost debugging time, its root cause, and the fix/principle
to apply next time. Skim the **Principles** at the bottom for the short version.

> Companion docs: `Firmware/Flavors/BareMinimal/NRF52_PORT_NOTES.md` (the nRF52
> port blow-by-blow), `doc/COMMANDS.md` (serial protocol).

---

## 1. RTOS thread stacks are both RTOS- *and* CPU-specific — never use raw literals

**Symptom:** On nRF52 the board flashed fine but printed nothing — not even the
first `Serial.println("HELLO...")` in `setup()`.

**Root cause:** Threads were created with ChibiOS-sized stacks copied as raw
numbers, e.g. `eris_thread_create(waThread1, 32, ...)`. On FreeRTOS the second
argument is the *actual byte allocation*: `32 bytes / 4 = 8 words`, far below the
~16+ words FreeRTOS writes just to lay down a task's initial register frame on
Cortex-M4F. `pxPortInitialiseStack` overran the heap → hardfault within ~1 ms of
thread creation. Because USB-CDC output is buffered (see §3), the queued "HELLO"
never transmitted before the crash, so the symptom *looked* like "serial dead."

**Fix:** Always use the RTOS-aware tier macros from `eris_rtos.h`, never magic
numbers:

| Tier | ChibiOS (M4F/Teensy) | FreeRTOS M0+ (SAMD21) | FreeRTOS nRF52 (M4F) |
|------|----------------------|-----------------------|----------------------|
| `ERIS_STACK_TINY`   | 32   | 256  | 256  |
| `ERIS_STACK_SMALL`  | 128  | 512  | 512  |
| `ERIS_STACK_MEDIUM` | 256  | 512  | 1024 |
| `ERIS_STACK_LARGE`  | 1024 | 1024 | 2048 |

The tiers are conditionally defined per platform. nRF52 (M4F + FPU + 256 KB RAM)
gets generous stacks; RAM-constrained M0+ stays tight. **Match the tier to the
work the thread does, not to a number that happened to work on Teensy.**

---

## 2. "Last-registered command stops working" == heap-tail corruption from a stack overflow

**Symptom:** Servo demo worked, then after a few `X` commands the firmware kept
running and `INFO`/`ON`/`OFF` still responded, but `X`/`Y` returned `What?`
(+ `<<ERROR>>Unsupported command<<ERROR>>`).

**Root cause:** `SerialCommand` stores its command list on the heap (grown via
`realloc` on each `addCommand`). `X`/`Y` are registered *last*, so they live at
the **tail** of that block. The `X` handler runs in the ReadSerial thread and does
`float args[16]` + I²C + **four float `Serial.print`s** — float→string formatting
is a notorious stack hog (~200–500 B). The 1 KB ReadSerial stack intermittently
overflowed into adjacent heap, clobbering the *end* of the command list first.

**Fingerprint to remember:** *Process stays alive, early-registered features keep
working, the most-recently-allocated thing breaks first* → almost always a
**stack overflow corrupting the heap** (FreeRTOS task stacks are `pvPortMalloc`'d
from the same heap as everything else).

**Fix:** Raise the offending thread's stack tier (here `ERIS_STACK_LARGE` → 2048
on nRF52). To *confirm* such a bug definitively, enable
`configCHECK_FOR_STACK_OVERFLOW` + a hook (needs a core-config edit).

---

## 3. nRF52 + TinyUSB: `Serial` is a library object, not part of the core

**Symptom A — link error on a bare sketch:**
`undefined reference to 'Serial'`, `Adafruit_USBD_CDC::begin`, etc. — even on the
stock `ASCIITable` example.

**Root cause:** With the TinyUSB USB stack (`-DUSE_TINYUSB`, the default on the
XIAO nRF52840), `Serial` is the `Adafruit_USBD_CDC` object defined in the
**Adafruit_TinyUSB_Arduino library**, not the core. The Arduino build only
compiles that library if some translation unit `#include`s one of its headers.

**Fix:** `#include <Adafruit_TinyUSB.h>` in the sketch. (Eris flavors get this
transitively because the core's `SPI.cpp` includes it `// for Serial`, and
flavors include `<SPI.h>` — but that's incidental; a flavor without SPI must
include it explicitly.)

**Symptom B — early prints vanish:** A crash shortly after a `Serial.println`
loses the message. USB-CDC `write()` only *queues* into a 256 B TX FIFO; the
bytes leave on the next USB frame (~1 ms). If you crash before that, the output
is gone — so missing early output does **not** mean the print line wasn't reached.

**Also note:** `Adafruit_USBD_CDC::write()` **blocks** (spins calling `yield()`)
while the TX FIFO is full and the host is connected. A slow/paused host reader can
stall whatever thread is printing.

---

## 4. nRF52 FreeRTOS is already running before `setup()`

The Seeeduino/Adafruit nRF52 core starts the FreeRTOS scheduler in `main()` and
runs `loop()` as a task at `TASK_PRIO_LOW (=1)`. Consequences baked into Eris:

- `eris_scheduler_start(fn)` on nRF52 just calls `fn()` — **do not** call
  `vTaskStartScheduler()` again.
- The classic `chBegin(start); while(true){}` idiom is **wrong** here: a
  `while(true){}` after start blocks the loop task forever and starves
  equal-priority Eris threads. Guard it: `#ifndef ERIS_USE_FREERTOS`.
- The core already defines `vApplicationGetIdleTaskMemory` /
  `...TimerTaskMemory`. A flavor defining them too → multiple-definition.
  Guard with `#if defined(ERIS_USE_FREERTOS) && !defined(NRF52_SERIES)`.
- ChRt's `SVC_Handler` clashes with the core's FreeRTOS one (patched weak in
  ChRt; and `eris_rtos.h` force-selects FreeRTOS on nRF52 so ChRt isn't linked).
- `configUSE_TIME_SLICING = 0`: equal-priority tasks only round-robin *when the
  running one blocks*. It works as long as every task sleeps/blocks, but it's
  fragile. For deterministic serial servicing the ReadSerial thread runs at
  `ERIS_NORMAL_PRIORITY+2` (above the loop/heartbeat tier; it still yields via
  its 10 ms sleep, so nothing starves).

---

## 5. Windows is case-insensitive — flavor filenames can shadow library headers

**Symptom:** `'SerialCommand' does not name a type` when the shared
`modules/serialcom.h` does `#include <SerialCommand.h>`.

**Root cause:** A flavor file named `serialcommand.h` (lowercase) collides with the
library header `SerialCommand.h` on Windows' case-insensitive filesystem, and the
sketch dir is early on the include path — so `<SerialCommand.h>` resolved to the
*flavor's* file.

**Fix:** Never name a flavor file a case-variant of a library header. Use a
distinct name: `servo_commands.{h,cpp}` (ErisServo, ErisServoDriver) or
`serialcommands.h` with the trailing 's' (BareMinimal).

**It is board-dependent, which makes it worse** (found writing ErisStepper,
2026-09-15). The same flavor, with a local `serialcommand.h`, compiled clean for
`teensy:avr:teensy36` and `Seeeduino:samd:seeed_XIAO_m0` and failed on
`esp32:esp32:esp32` with exactly the symptom above — the include-path order the
core builds differs, so the shadow only wins on some targets. A flavor can
therefore carry this bug for years and look fine. Note `ErisDCMotor` still has a
`serialcommand.h`; only the `.cpp` may keep that name, because discovery
resolves headers, not sources.

---

## 6. Don't keep a local copy of a promoted shared module

**Symptom:** `multiple definition of 'SineWave::start()' / 'SineWave::buffer'`.

**Root cause:** ErisServoDriver had a local `sinewave.cpp` *and* included
`<modules/sinewave.h>`; the shared `eriscommon/src/modules/sinewave.cpp` also
compiles whenever the library is used → two definitions of the same symbols.

**Fix:** Once a module is promoted to `eriscommon/src/modules`, delete the
flavor's local copy and include only the shared header. Use `tools/check_drift.py`
to spot lingering duplicates.

---

## 7. One thread owns the I²C bus; size the bus to the update budget

When adding the all-servo demo, the right move was to drive it **from inside the
existing `SmoothServo_T` control loop** (already the sole periodic I²C writer),
*not* a second thread. `Wire`/TWIM is **not reentrant** — two threads issuing
`pwm.setPWM()` concurrently corrupts bus transactions.

**Budget the bus:** a full `NUM_SERVOS=16` update every `SERVO_SMOOTH_LOOP_MS=5 ms`
needs ~16 × ~0.6 ms ≈ 10 ms at 100 kHz — it won't fit. Bumping the PCA9685 to
400 kHz (`Wire.setClock`, fast-mode) drops it to ~2–3 ms, comfortably inside the
tick. **When you fan a periodic task out across N devices, check N × per-device
time against the loop period.**

Pattern used: the demo writes `current == target` for its channels so the normal
smooth-stepping pass skips them (no double-write), and keeps state in sync so
servos hold position (don't snap) on `DEMO_OFF`.

---

## 8. Verify headlessly with the IDE's bundled `arduino-cli`

No separate toolchain needed — Arduino IDE 2.x ships one:
`<...>/arduino-ide/resources/app/lib/backend/resources/arduino-cli.exe`

```sh
arduino-cli compile -b Seeeduino:nrf52:xiaonRF52840 <flavor-dir>
arduino-cli compile --clean ...            # when stale .o cache lies (e.g. after a rename)
arduino-cli compile --format json ...      # to see actually-linked "used_libraries"
```
After a file rename, **clean-build**: the per-sketch cache can keep an old `.o`
and produce phantom multiple-definition errors.

**Two `arduino-cli`-only gotchas** — PlatformIO pins libraries via `lib_deps`, so
it sidesteps both; these bite only the bundled-CLI path:

- *Conditional includes defeat library discovery.* `arduino-cli` decides which
  libraries land on the include path by scanning `#include`s. A header reached
  only inside a conditional branch — e.g. `<ChRt.h>` behind the `#else` of
  `ERIS_USE_FREERTOS` in `Eris.h` — can be missed, so `eris_rtos.h`'s
  `__has_include(<ChRt.h>)` is false and you get *"No supported RTOS detected"*
  even on a Teensy. Force it for a one-off build with
  `--build-property compiler.cpp.extra_flags=-DERIS_USE_CHIBIOS` (or
  `--library <path-to-ChRt>`), or just build that flavor with PlatformIO.
- *`__has_include`-guarded modules need a literal include to bootstrap the path.*
  Same root cause, nastier shape: the BLE transport in eriscommon is gated on
  `__has_include(<bluefruit.h>)`, but `bluefruit.h` is only on the include path
  if some source *literally* `#include`s it. Guarding the include behind the very
  `__has_include` that's supposed to find it is circular — the path is never
  added, the guard is always false, and the module compiles to **nothing**. The
  build *succeeds* and the firmware just never advertises (no error to chase).
  Fix: the flavor's `.ino` must carry a real `#include <bluefruit.h>` (under
  `#ifdef ERIS_USE_BLE`); that puts Bluefruit on the path for every TU, flipping
  the guard true. The literal include is mandatory even though the sketch never
  calls Bluefruit directly. Symptom that fingerprints this: referencing the
  guarded namespace (e.g. `BLETransport`) from the sketch fails with
  *"'BLETransport' has not been declared"* while the same code "compiled fine"
  before the reference existed.
- *Duplicate library copies.* If `eriscommon` exists twice on the search path
  (the `C:/git/ArduinoLibraries` working copy **and** the IDE sketchbook
  `Documents/Arduino/libraries`), `arduino-cli` may pick either, inconsistently
  between flavors. Keep one source of truth — make the sketchbook copy a junction
  to the git copy, not a second real directory.

---

## 9. New-flavor boilerplate lives in eriscommon — don't paste it

The three things every flavor used to copy into its `.ino` were exactly the three
things the lessons above say are easy to get wrong. They are now shared, so a new
flavor should **use the helper, not re-paste the pattern**:

- **`ERIS_RUN(start)`** (in `eris_rtos.h`) — replaces the
  `eris_scheduler_start(start); #ifdef ERIS_USE_FREERTOS … #else while(true){} #endif`
  idiom. It does the right per-RTOS thing: spins after `chBegin` on ChibiOS,
  returns on nRF52 so the loop task yields (§4), starts the scheduler on other
  FreeRTOS. One line, no `#ifdef` to get wrong.
- **`Heartbeat::start()`** (in `<modules/heartbeat.h>`) — the sleep-only
  "Thread1" placeholder, promoted to a module with an `ERIS_STACK_TINY` stack
  (§1). Call it instead of re-declaring `waThread1`/`Thread1`. Drop it entirely
  if the flavor has its own real heartbeat (e.g. ErisServoHand blinks an LED).
- **FreeRTOS static-alloc callbacks** — `vApplicationGet{Idle,Timer}TaskMemory`
  now live once in `eris_freertos_hooks.cpp`, guarded `ERIS_USE_FREERTOS &&
  !NRF52_SERIES` (SAMD21 needs them, `configSUPPORT_STATIC_ALLOCATION=1`; nRF52's
  core already provides them, §4). Do **not** redefine them in a flavor — that's
  a multiple-definition (§6 in spirit).

A minimal flavor `start()`/`setup()` is now just `Heartbeat::start(); Error::start();
… SerialCom::start();` and `ERIS_RUN(start);`. See BareMinimal for the template.

---

## 10. A flavor left behind by a library refactor fails only at *link* time

**Symptom:** The `Eris` flavor compiled file-by-file but the link cascaded —
fix one error, hit the next: `undefined reference to eriscommon::printText`, then
undefined `strbuffer`, then `multiple definition of SineWave::start()`.

**Root cause:** It had drifted from a refactored `eriscommon`: `printText()` was
dropped in favour of `print()/println()` (the declaration lingered in the header
with no definition — a link error, not a compile error), `t0` moved into the
library, and `sinewave` was promoted to a shared module while the flavor kept its
local copy (§6). None of these show up in a per-translation-unit compile — only a
full link surfaces removed symbols and duplicate definitions.

**Fix / principle:** After changing `eriscommon`'s public surface (renaming or
removing a function, promoting a module, moving a global), **full-*link* every
flavor that touches it**, not just compile. A green per-file compile hides exactly
these failures. When a flavor lags, the repair is usually: swap the dead API for
its replacement, define any flavor-owned globals it expects (`strbuffer` lives in
the flavor, e.g. in the `.ino`), and delete local copies of promoted modules.

---

## 11. A header included unconditionally must compile to nothing when its feature is off

**Symptom:** `fatal error: SD.h: No such file or directory` from `eris_sd.h` on a
board without SD — even with `SDCARD` disabled.

**Root cause:** Every flavor's `Eris.h` includes `<eris_sd.h>` *unconditionally*,
but the header did `#include <SD.h>` and declared a template whose signature names
the SD `File` type — both outside any guard. A function template's parameter types
are parsed even when it is never instantiated, so guarding only the `#include`
isn't enough. It also had a half-written include guard: `#ifndef ERIS_SD_H` with
no matching `#define`, so it never actually guarded.

**Fix:** Wrap the **entire body** (includes *and* the template) in `#if SDCARD`,
matching the same idiom flavors already use for their own SD code, and pair every
`#ifndef GUARD` with a `#define GUARD`. A feature header pulled in by everyone must
be a true no-op when its feature is off.

---

## 12. BLE saturation shreds COBS framing — back-pressure the stream, don't drop bytes

**Symptom:** Over the BLE (nRF52 NUS) transport the host decoded **0 of N** COBS
frames — every frame "bad" — while the same firmware over USB serial was fine. A
raw byte dump showed clean-looking float data and the right `0x00` delimiters, but
each frame's leading COBS code byte (e.g. `0x83` = 131) pointed past the end of a
truncated ~42 B frame.

**Root cause:** Two compounding problems. (1) The link was massively
under-provisioned: MTU stuck at the 23 B default (20 B payload) and a slow
connection interval delivered only ~180 B/s, while SineWave at 100 Hz × 8 B plus
framing wanted ~800+ B/s. (2) `Packet::send` → `PacketSerial::send` writes each
COBS packet in one `write()` and **never checks the return value**; Adafruit
`BLEUart::write()` returns *short* when the SoftDevice notification queue is full,
so the tail of every oversized packet was silently dropped — corrupting the COBS
frame. The `0x83 44 10 …` header decodes correctly: it is a *valid* code byte for a
130 B zero-free `D`-packet (`'D'` + count `16` + 16×8). The encoder was never
wrong; the wire was lossy.

**Confirmation method:** Throttle the *generation* rate (the real byte-rate driver,
not the streaming period — batching changes packet size, not total volume) to
10 Hz. Frames jumped to 13/21 ok and a decoded packet showed real 99.7 ms-spaced
timestamps. Decreasing per-packet `count` (`16,16,16,6,3`) was the startup backlog
draining: big packets truncated, small ones survived → saturation proven.

**Fix (in `ble_transport.cpp`):**
- `Bluefruit.configPrphBandwidth(BANDWIDTH_MAX)` **before** `begin()` → MTU 247, so a
  whole packet fits one notification.
- `Bluefruit.Periph.setConnInterval(6, 12)` → request a 7.5–15 ms interval.
- A `Stream` wrapper (`BackpressureUart`) whose `write()` **loops until every byte is
  accepted**, yielding (`delay(1)`) between attempts so the radio drains its queue on
  the next connection event; bails only on disconnect / multi-second stall. This is
  the byte-loss cure — PacketSerial's unchecked single `write()` needs a stream that
  can't truncate.
- Drop `bufferTXD()`: with a large MTU it buys nothing and would hold a packet's
  sub-MTU tail until the next packet, stalling command replies.

**Result:** MTU 247, 131 B notifications, **30/30 frames ok**, ~102 Hz on the wire,
0 dropped packets at full 100 Hz.

---

## 13. Memory & starvation model — what actually bounds this firmware

A tour of where RAM and CPU time go, and the failure modes to expect. Ties together
§1 (stacks), §2 (heap-tail corruption) and §4 (nRF52 scheduling).

**Stacks (the primary hazard).** Thread stacks are sized by `ERIS_STACK_*` tiers whose
byte value changes per **RTOS *and* CPU** — a literal that fits Teensy overflows nRF52:

| Tier | ChibiOS/Teensy M4F | FreeRTOS/SAMD21 M0+ | FreeRTOS/nRF52 M4F |
|------|--------------------|---------------------|--------------------|
| TINY / SMALL / MEDIUM / LARGE | 32 / 128 / 256 / 1024 | 256 / 512 / 512 / 1024 | 256 / 512 / 1024 / 2048 |

Core threads always present: Heartbeat (TINY), Error (SMALL), SineWave (MEDIUM),
ReadSerial (LARGE), StreamSerial (LARGE) — ≈ 5.9 KB (nRF52) / 3.3 KB (SAMD21) /
2.5 KB (Teensy) before any sensor thread. ReadSerial is LARGE because **float→string
formatting (`Serial.print(aFloat)` ≈ 200–500 B) + I²C in a command handler** is the
top stack hog (§2). Two gotchas:
- **On ChibiOS the `eris_thread_create` stack arg is *ignored*** — the real size is
  the `ERIS_THREAD_WA(name, size)` declaration (`chThdCreateStatic` uses
  `sizeof(wa)`). On FreeRTOS it's the reverse (`ERIS_THREAD_WA` is empty; the
  create-call arg is used). **To resize portably, change *both*.** (This is how the
  ErisBiom2 feature thread hid a 256 B stack behind a create call — fixed 2026-07-01.)
- `sizeof(waXxx_T)` as the stack arg only compiles on ChibiOS (that symbol doesn't
  exist under FreeRTOS) → those flavors are Teensy-only.

**Heap is a boot-time budget, not runtime churn.** The print path is char-buffer
based (no `String` in the hot path) so there's **no runtime fragmentation**. But on
FreeRTOS everything dynamic is `pvPortMalloc`'d **once at boot** from
`configTOTAL_HEAP_SIZE`: every task's stack+TCB (`xTaskCreate`), every `ErisBuffer`
queue (`xQueueCreate`, depth `MEMBUFFERSIZE`=64), semaphores, and the `SerialCommand`
list. The real ceiling is **SAMD21 (32 KB RAM)** — a few threads + buffers is most of
the heap. Worst part: nobody checked the return, so an out-of-heap `xTaskCreate` /
`xQueueCreate` returns **NULL silently** → the thread/buffer just never runs, looking
like a dead feature. Fixed 2026-07-01: `eriscommon::checkAlloc(handle != NULL, name)`
+ `eris_mb_valid(mb)` now report a `<<FATAL>>` line + `Error::MEMORY` at every core
creation and in `ErisBuffer::init()`.

**Backpressure is lossy, not blocking.** `ErisBuffer::append` never stalls a producer —
when full it drops the **oldest** sample and bumps `droppedCounter` (`missed()`). So a
starved *consumer* shows up as counted dropped samples, never a hang.

**Scheduling / starvation.** Priority map (FreeRTOS `NORMAL = tskIDLE+1`):
```
NORMAL+3 StreamSerial   > +2 ReadSerial   > +1 SineWave/sensors/servo/FSR   > NORMAL Heartbeat/Error   > 0 idle
```
On nRF52 `configUSE_TIME_SLICING = 0` → equal-priority tasks do **not** round-robin; a
task holds the CPU until it blocks/yields. So **every thread must `eris_sleep_ms()`**,
and a busy loop at ≥ its neighbours' priority starves them (that was §4). Comms sit
*above* acquisition on purpose (link stays responsive; starved sensors just drop). The
BLE back-pressure loop (§12) is safe here because it yields via `delay(1)`. **Don't put
a compute thread above the comms threads** — ErisBiom2's feature thread was at
`NORMAL+5` (highest in the system), so a heavy feature pass preempted and stalled the
stream; moved to `NORMAL+1` (2026-07-01). Separately, the BLE SoftDevice runs at a HW
IRQ priority above all tasks (can't be starved), but `ErisBuffer::append`/`FetchData`
scan up to `MEMBUFFERSIZE` entries inside a `taskENTER_CRITICAL` — keep `MEMBUFFERSIZE`
modest on BLE builds or that IRQs-off window grows.

---

## 14. ESP32 is a third RTOS dialect, not "just FreeRTOS"

`eris_rtos.h` treated FreeRTOS as one thing. ESP32 (arduino-esp32 3.3.0 /
ESP-IDF v5.5) breaks that in five separate places, and four of them fail
*silently or at runtime* rather than at the build:

1. **Detection never fires.** IDF's header is `<freertos/FreeRTOS.h>`; the bare
   `__has_include(<FreeRTOS.h>)` probe is false, so the old header fell through
   to `#error "No supported RTOS detected"`. ESP32 must be detected by
   `defined(ESP32)` *before* the `__has_include(<ChRt.h>)` probe — ChRt is
   Cortex-M only, and a copy installed for the Teensy flavors would otherwise
   win the race and produce nonsense.
2. **`taskENTER_CRITICAL()` does not exist in the no-arg form.** IDF's FreeRTOS
   is SMP and wants a `portMUX_TYPE` spinlock. Use `portENTER_CRITICAL_SAFE()`
   (picks ISR vs task automatically — `ErisBuffer::append()` is called from
   both) with **one global mux defined in a `.cpp`**. A `static` mux per
   translation unit compiles fine and silently protects nothing across TUs.
3. **Stack depth is in BYTES, not words.** `xTaskCreate(..., bytes/4, ...)` —
   the portable idiom everywhere else — hands every thread a quarter of the
   stack it asked for. Pass the tier through unscaled on ESP32, and size the
   tiers several times larger than the Cortex-M ones (~768 B is IDF's own floor;
   anything touching float printf wants 4 K).
4. **The scheduler is already running**, exactly as on nRF52: `ERIS_RUN()` must
   call `start()` and *return*. Additionally `loop()` must yield — it shares a
   core with the Eris threads and a busy `loop()` starves the idle task that
   feeds the task watchdog, which reboots the board.
5. **It is dual-core.** Eris's buffers and critical sections assume single-core.
   Pin every Eris thread (`xTaskCreatePinnedToCore`, `ERIS_ESP32_CORE`, default
   core 1) so the concurrency model matches the other ports. Single-core parts
   (S2/C3/C6/H2) need core 0 — key off `CONFIG_FREERTOS_NUMBER_OF_CORES`.

Also: `xTaskCreatePinnedToCore` lives in `freertos/idf_additions.h`. FreeRTOS.h
includes it implicitly, but only under `ESP_PLATFORM` and only "for
compatibility reasons" — IDF's own comment schedules that for removal in v6.0.
Include it explicitly.

**ESP8266 is not portable to.** Its Arduino core is bare-metal with no RTOS;
`eris_rtos.h` now says so with an explicit `#error` instead of a confusing
cascade of missing types.

### Two latent bugs ESP32 exposed in existing code

Both had been dormant on every other board:

- **`Eris.h` tested `ERIS_USE_FREERTOS` before `eris_rtos.h` was included** —
  i.e. before detection had run — so it *always* took the `#else` branch and
  `#include <ChRt.h>`. It only ever "worked" because ChRt is installed for the
  Teensy flavors. Include `<eris_rtos.h>` first, then branch on the result.
  Fixed in `ErisESP32/Eris.h`; **the other flavors still have the old order.**
- **`eriscommon::printText()` was declared in `eriscommon.h` but never
  defined**, and `error.cpp` calls the `F()` overload four times. The reference
  only resolved on cores where `--gc-sections` happened to discard those
  handlers. Now implemented (emits a TEXT packet, matching `error.cpp`'s use of
  TEXT as the resting packet type).

### Per-flavor pin mapping: `eris_board.h`

Porting a flavor's *pins* is a bigger job than porting its RTOS calls, and the
trap is not the pin numbers -- it is the `A*` aliases. The ESP32 variant defines
`A0, A3..A7, A10..A15`: there is **no `A1`, `A2`, `A8` or `A9`**. Nine flavors
used `A1`, so they fail to compile. And of the aliases that do exist, `A10..A15`
are ADC2 pins whose `analogRead()` returns garbage the moment WiFi is enabled.

`eriscommon/src/eris_board.h` holds the board-wide facts (it is a header, so a
flavor's `configuration.h` can include it -- unlike eriscommon's `.cpp` files,
which never see flavor defines):

- `ERIS_BOARD_ESP32 / TEENSY / NRF52 / SAMD / OTHER` -- detection
- `PIN_LED`, `ERIS_SERIAL_BAUD`, `ERIS_I2C_BEGIN()` -- the universal three
- `ERIS_ADC(n)` -- channel *n* -> a real, WiFi-safe pin (ESP32: ADC1 only, six
  channels 36/39/32/33/34/35; Teensy: `A0..A9`). **Use this instead of `A1`.**
- `ERIS_PIN_ASSERT_OUTPUT(p)` / `ERIS_PIN_ASSERT_ADC1(p)` -- `static_assert`s
  that reject the three silent-failure classes at build time: flash pins
  (6..11, board will not boot), input-only pins (34..39, `digitalWrite` is a
  no-op), and ADC2 pins. They compile to nothing on non-ESP32 boards.

Everything is `#ifndef`-guarded, so a flavor overrides one value by defining it
*before* the include. Flavor-specific signals stay in `configuration.h` behind
`#if defined(ERIS_BOARD_ESP32)`, because those are wiring decisions.

Hard constraint worth knowing up front: **ESP32 has six WiFi-safe analog
inputs**, not eight. Flavors wanting `A0..A7` must drop to six channels, give up
WiFi, or add an external ADC.

Reference conversions: `Eris` (analog path) and `ErisESP32` (trivial path).

### Verifying an ESP32 build without arduino-cli or a PlatformIO download

The Board Manager core carries everything needed to compile out-of-tree — no
toolchain download required:

```
CORE=$LOCALAPPDATA/Arduino15/packages/esp32
# Layout changed in core 3.3.11: per-chip `esp32-libs`, no idf-release_* level.
# Older cores used $CORE/tools/esp32-arduino-libs/idf-release_v5.5-*/esp32
IDF=$CORE/tools/esp32-libs/<core-version>
$CORE/tools/esp-x32/*/bin/xtensa-esp-elf-g++.exe -c file.cpp   $(cat $IDF/flags/cpp_flags) -iprefix $IDF/include/ $(cat $IDF/flags/includes)   -I$IDF/qio_qspi/include -I$CORE/hardware/esp32/*/cores/esp32   -I$CORE/hardware/esp32/*/variants/esp32 -DESP32 -DARDUINO_ARCH_ESP32
```

Compile every `eriscommon` + flavor TU this way, then `nm -C *.o`, subtract
defined symbols from undefined ones, and grep for Eris-level names — that catches
link breakage (like `printText`) without linking the whole IDF. Preprocessing the
old and new header side by side with `-E -nostdinc` against stub headers proves a
shared-header change is a no-op for the boards you did not touch.

## 15. Shared-name headers: the duplication cleanup, and what it exposed

Consolidation pass (2026-08-21). Four things were duplicated into every flavor
and one of them was actively broken:

- **`strbuffer`** was declared `extern char strbuffer[STRBUFFERSIZE]` in 18
  `Eris.h` files and **defined in only three flavors**. `ErisADS1299`,
  `ErisNextFlex` and `ErisTapok2` `sprintf()` into it from their SD-record
  handlers -- those only ever linked because `--gc-sections` discarded the
  handler. Exactly the `printText` failure mode from §14. It now lives in
  `eriscommon` (`ERIS_STRBUFFER_SIZE`, 128 = the largest any flavor asked for);
  flavors must not re-declare it. `ErisNextFlexArray` additionally had a
  file-local `static char strbuffer[128]` shadowing the extern -- renamed
  `sdWriteBuf`.
- **`mtxhb`** was declared in 10 `Eris.h` files and referenced **zero** times.
  Deleted.
- **`Eris.h` itself** collapsed onto `<eris_flavor.h>`, which owns the include
  order that §14 got wrong (eris_rtos.h first, then the guarded ChRt, then
  Arduino/eriscommon/eris_streaming). Flavor `Eris.h` files went from 17-30
  lines to 10-14 and are now uniform.
- **`STRINGIFY`/`TOSTRING`** were pasted into 18 `configuration.h` files. They
  live in `eris_board.h`, which `configuration.h` includes -- note it must be
  *there* and not in `eris_flavor.h`, which is included too late to help build
  `FIRMWARE_INFO`.

### Same-name headers are a Windows trap (twice over)

Windows filename lookup is **case-insensitive**, so a header in the sketch
folder can hijack a library header whose name differs only in case or path:

1. Each flavor's `customtypes.h` did `#include <customtypes.h>` to reach
   eriscommon's file *of the same name* -- self-shadowing whenever the sketch
   dir precedes the library on the include path. eriscommon's is now
   `eris_customtypes.h`. Fixed.
2. **Still open:** 14 flavors have `serialcommand.h`, which collides
   case-insensitively with the `SerialCommand.h` *library*. Put the sketch dir
   first on the include path and `eriscommon/modules/serialcom.h`'s
   `#include <SerialCommand.h>` resolves to the flavor's file, and the whole
   command layer fails with `'SerialCommand' does not name a type`. It builds
   today only because the toolchain happens to search the library path first.
   `BareMinimal` and `ErisESP32` already dodge it by spelling the file
   `serialcommands.h` (plural) -- that is the fix for the other 14.

**Rule: never name a flavor header the same as a library header, in any
casing.** These bugs are invisible until an include path is reordered.

### CI

`.github/` had been empty -- no workflow at all, which is why every bug in §14
and §15 survived. `compile.yml` now builds a matrix (ErisESP32 + ErisServoDriver
on esp32, ErisServo + ErisServoDriver on teensy36) and runs `check_drift.py`
informationally. It needs an `ARDUINO_LIBS_TOKEN` secret with read access to the
separate `ArduinoLibraries` repo where `eriscommon` lives.

## 16. `__has_include()` cannot discover an Arduino library

The 2026-08-21 consolidation moved `#include <ChRt.h>` out of every flavor's
`Eris.h` and into `eris_flavor.h`, behind `#ifdef ERIS_USE_CHIBIOS`. That looks
strictly better -- the ESP32 fix in §14 required exactly that ordering. It
silently broke **every ChibiOS flavor in the Arduino IDE**, while PlatformIO
kept working, which is why it went unnoticed.

The Arduino builder discovers libraries by **failing**: it preprocesses the
sketch, catches `fatal error: X.h: No such file or directory`, maps `X.h` to a
library, adds that library's `src/` to the include path, and retries. The loop
is driven entirely by unresolved-include errors.

`__has_include(<ChRt.h>)` never produces that error. It evaluates to 0 and moves
on. So on the first pass ChRt is not on the include path, the probe is false,
detection falls through to `#error "No supported RTOS detected"`, and the build
dies **before the builder ever learns it needed ChRt** -- with an error message
pointing at the RTOS layer rather than at library discovery.

Before the consolidation the unconditional `#include <ChRt.h>` in each `Eris.h`
was doing double duty: selecting the RTOS *and* feeding the discovery loop. Only
the first job was obvious, so only the first was preserved.

**Rule: a probe can test for a library that is already on the path; only a
literal `#include` can put it there.** Detection that must *find* a library has
to key off something known on the first pass -- a core-supplied `-D` macro --
and then include the header literally:

```c
#elif defined(TEENSYDUINO)
  #include <ChRt.h>                    // literal: makes the builder add ChRt
  #define ERIS_USE_CHIBIOS
#elif defined(ARDUINO_ARCH_SAMD)
  #include <Seeed_Arduino_FreeRTOS.h>
  #define ERIS_USE_FREERTOS
```

`__has_include` probes stay as the fallback for cores that bundle their RTOS
(nRF52, ESP32) and for PlatformIO, which resolves `lib_deps` up front and never
needed the discovery loop.

This is the same failure as the `bluefruit.h` rule in the BLE work (§12): the
flavor `.ino` must *literally* include it or Arduino never adds the path. That
was written down as a BLE quirk. It is not -- it is a general property of the
Arduino build system, and it applies to every optional dependency Eris has.

**Corollary for testing:** PlatformIO and the Arduino IDE resolve dependencies
by different mechanisms, so a green PlatformIO build proves nothing about the
IDE. Any change to conditional includes needs a first-pass preprocess check with
*only* the core + sketch dir on the include path.

## 17. Arduino's Print.h owns BIN, DEC, HEX and OCT

`enum Mode : uint8_t { BIN = 0, ASCII = 1 };` does not compile on Teensy. Print.h
defines `BIN` as a macro (it is the second argument to `Serial.print(x, BIN)`),
so the enumerator expands to `2 = 0` and the error lands on *Print.h*, several
frames from the header that actually has the problem:

```
Print.h:44:13: error: expected identifier before numeric constant
eris_ascii.h:57:1: error: 'Mode' does not name a type; did you mean 'mode_t'?
```

`DEC`, `HEX`, `OCT` and `BIN` are all taken, in every Arduino core. They are
plain `#define`s, so no namespace or enum class protects you -- the preprocessor
runs first. The Eris ASCII mode enum is `{ Binary, Text }` for this reason; the
user-facing `S_MODE` command still spells them `BIN` and `ASCII`, because the
command parser compares strings, which the preprocessor cannot touch.

The general rule: a name that is a macro anywhere in the include graph cannot be
an identifier anywhere in the translation unit. When an error appears *inside a
core header* right after you add an enum or a variable, suspect a macro
collision before suspecting the core.

## 18. On SAMD FreeRTOS, `start()` ran with interrupts permanently masked

**Symptom:** SAMD21 prints `HELLO, This is Eris` from `setup()` and freezes.
No further output, no threads, no assert. Nothing in `start()` ever completes.
Found bringing up ErisStepper, 2026-09-15.

**Root cause:** `eris_scheduler_start()` used to be

```c
#define eris_scheduler_start(fn)  do { fn(); vTaskStartScheduler(); } while(0)
```

so `start()` ran on the bare metal, before the scheduler. That is fatal on this
port because of how it handles critical sections (`tasks.c`, under
`portCRITICAL_NESTING_IN_TCB`):

```c
void vTaskEnterCritical( void ) {
    portDISABLE_INTERRUPTS();                       /* ALWAYS */
    if( xSchedulerRunning != pdFALSE ) { ...nesting++... }
}
void vTaskExitCritical( void ) {
    if( xSchedulerRunning != pdFALSE ) { ...nesting--; if 0 -> portENABLE_INTERRUPTS(); }
    /* scheduler not running: does NOTHING */
}
```

Enter always masks interrupts; exit only unmasks them once the scheduler is
running. **So the first critical section taken before `vTaskStartScheduler()`
masks interrupts for good** -- and `xTaskCreate()` takes one internally, in
`prvAddNewTaskToReadyList()` (`tasks.c:1072`).

Every Eris `start()` opens with `Heartbeat::start()`. Interrupts therefore died
on line one of `start()` and stayed dead for the whole of it:

- SysTick stopped, so `millis()`, `micros()` and `delay()` stopped advancing --
  any timeout loop spins forever and `delay()` never returns;
- UART, I2C and USB interrupts stopped firing, so interrupt-driven driver
  bring-up and `Serial.flush()` hang.

ErisStepper froze in `delay(10)` inside `TMC::begin()`. Flavors whose `start()`
only creates threads never noticed, which is why this survived this long.

**The corroborating clue:** disabling the flavor's threads made the firmware get
*further* (all the way to `bootDefaults()`), not less far. Removing the
`xTaskCreate` calls removed the thing that was masking interrupts. If chopping
out code makes a hang move later, suspect global machine state, not the code
you removed.

**Fix:** `start()` now gets a task of its own and the scheduler runs it --
which is what the other three platforms already did (ChibiOS runs it as the
main thread via `chBegin()`; nRF52/ESP32 call it from an already-scheduled
task). All four now agree: **`start()` always runs with the scheduler up and
interrupts enabled.** The bootstrap task runs above every Eris thread so
`start()` still finishes before its threads run, and deletes itself afterwards.

This also retires a hazard worth remembering on its own: `eris_sleep_ms()` is
`vTaskDelay()`, which walks `pxCurrentTCB` with no NULL guard
(`prvAddCurrentTaskToDelayedList`), so calling it before the scheduler hard-
faults on Cortex-M0+. Legal under ChibiOS, fatal here -- another reason
`start()` had no business running bare-metal.

**Fingerprint:** dies after the last `Serial.print` in `setup()`, with nothing
from `start()`. Note §3 -- with USB CDC a crash can swallow output that was
already queued, so "silent" does not prove "got no further".

---

## 19. `vTaskDelayUntil()` takes a PERIOD, not a deadline -- and asserts on 0

**Symptom:** a 1 ms periodic thread kills the board instantly; a 4 ms one
"works" but runs at the wrong rate. On SAMD21 the config's `configASSERT`
prints `ASSERT: <fn> :#<line>` and calls `assertBlink()`.

**Root cause:** `eris_sleep_until()` in `eris_rtos.h` was

```c
TickType_t period = *next - xTaskGetTickCount();
vTaskDelayUntil(next, period);          // WRONG
```

against callers that had already done `next += ERIS_MS_TO_TICKS(p)` and expect
"sleep until this absolute time" (the ChibiOS `chThdSleepUntil()` contract).
Three things go wrong at once:

- `vTaskDelayUntil()` advances `*next` itself, so the increment is counted
  twice and the period drifts;
- if the tick has already *reached* the deadline, `period == 0` and
  `configASSERT( xTimeIncrement > 0U )` at the top of `vTaskDelayUntil()` fires
  -- firmware dead;
- if the tick has already *passed* it, the unsigned subtraction underflows to
  ~2^32 and the task sleeps until the tick counter wraps.

A 4 ms thread (ErisMPU) usually finishes its work inside the period and only
suffers the drift, which is why this survived unnoticed. A 1 ms thread hits the
`== 0` case on essentially every tick.

**Fix:** honour the absolute-deadline contract with a signed comparison, which
also handles tick wrap:

```c
int32_t remaining = (int32_t)(*next - xTaskGetTickCount());
if (remaining > 0) vTaskDelay((TickType_t)remaining);
```

**Corollary -- the starvation trap.** "Return immediately if the deadline
passed" is right (ChibiOS does the same), but it means a thread whose work
overruns its period never sleeps at all. At a priority above everything else
that locks the whole firmware. A periodic thread that runs hot should resync
rather than try to catch up: see `paceTick()` in ErisStepper's `motion.cpp`.

---

## 20. `ErisBuffer` used before `init()` = `ASSERT: uxQueueMessagesWaiting`

**Symptom:** `ASSERT: uxQueueMessagesWaiting :#1985` on SAMD21.

**Root cause:** that line is `configASSERT( xQueue )` -- the queue handle is
`NULL`, i.e. `xQueueCreate()` was never called for it. `ErisBuffer::init()` is
what creates it, and `init()` lives inside the sensor's `start()`. So the way
in is mundane: **select a streaming feature whose sensor thread is not
started** (comment out a `start()` call, or have `bootDefaults()` name a
feature that never came up) and the first `StreamSamples()` asserts.

`ErisBuffer` already had a `ready` flag for exactly this, declared and never
read. It is now set by `init()` and checked by `append`, `FetchData`, `missed`
and `clear`, so an uninitialised buffer streams nothing instead of stopping the
firmware.

---

## Principles (the short version)

1. **No magic numbers for stacks.** Use `ERIS_STACK_*`; remember the value means
   different things per RTOS/CPU.
2. **Float `printf` + I²C in a thread = give it a big stack.** Especially M4F.
3. **"Newest feature breaks first, process survives" = heap/stack overflow.**
4. **Buffered USB-CDC output ≠ transmitted.** Don't trust missing early prints.
5. **One owner per shared peripheral** (I²C/SPI/Serial). Budget periodic work
   against the loop period.
6. **Filenames can shadow library headers on Windows.** Keep them distinct.
7. **Promoted module ⇒ delete the local copy.** Watch for drift.
8. **nRF52 FreeRTOS is already running** — don't restart the scheduler, don't
   block the loop task, mind the core-provided callbacks.
9. **Compile-verify every change** with the bundled `arduino-cli`; clean-build
   after renames.
10. **Use the shared helpers in new flavors** — `ERIS_RUN(start)`,
    `Heartbeat::start()`, and the library's FreeRTOS callbacks. Don't re-paste
    the scheduler/heartbeat/callback boilerplate.
11. **After an `eriscommon` API change, full-*link* every dependent flavor** —
    per-file compile hides removed-symbol and duplicate-symbol errors.
12. **An unconditionally-included header must no-op when its feature is off** —
    guard the whole body (template signatures too), and never leave a half-written
    include guard.
13. **A lossy transport needs back-pressure, not a faster encoder.** PacketSerial's
    single unchecked `write()` truncates on a full BLE queue; wrap the stream so
    `write()` can't drop a packet's tail, and size the link (MTU + conn interval) to
    the data rate. COBS "all frames bad" = byte loss, not a framing bug.
14. **Know the memory & starvation budget.** Stacks are RTOS+CPU-specific and, on
    ChibiOS, set by the `ERIS_THREAD_WA` decl not the create arg (change both).
    FreeRTOS task stacks + queues are heap-allocated once at boot and fail *silently*
    on OOM — `checkAlloc`/`eris_mb_valid` them. Threads must yield (`eris_sleep_ms`);
    keep comms above acquisition and never a compute thread above comms.
15. **ESP32 is a separate RTOS dialect.** IDF headers are `<freertos/...>`,
    critical sections need a global spinlock, stack depths are in *bytes*, the
    scheduler is already running, and it is dual-core — pin Eris threads to one.
    ESP8266 has no RTOS at all.
16. **Never give a flavor header a library header's name** (any casing --
    Windows lookup is case-insensitive). `customtypes.h` and `serialcommand.h`
    both collide; symptoms appear only when the include order changes.
17. **`__has_include()` cannot discover an Arduino library** -- the builder only
    adds a library when a *literal* include fails. Probe to test, include to
    find. A green PlatformIO build does not prove the Arduino IDE builds.
18. **Arduino's `Print.h` owns `BIN`/`DEC`/`HEX`/`OCT`** as macros -- never use
    them as identifiers. The error surfaces inside the core header, not yours.
