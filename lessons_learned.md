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
