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

### 7. `while(true){}` blocking after `setup()` (CURRENT — not fully resolved)

**Status:** The scheduler and `while(true)` fix are applied. Threads are created and `start()` completes. However, serial commands are not being responded to after "Serial Commands are ready" prints.

**Likely cause:** The ReadSerial thread is created at `ERIS_NORMAL_PRIORITY` (= `tskIDLE_PRIORITY + 1` = 1 on nRF52). With `configUSE_TIME_SLICING=0`, equal-priority tasks don't pre-empt each other. The loop task at priority 1 calls `eris_sleep_ms(10000)` which should yield — but newly created tasks at the same priority may not run until the creating task blocks. The thread may need a higher priority or an explicit `taskYIELD()` after thread creation.

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

## Remaining Issue

Serial commands not responded to on nRF52 after "Serial Commands are ready". Thread creation succeeds but `sCmd.readSerial()` thread may not be scheduled. Possible fixes:
- Raise ReadSerial thread priority above `ERIS_NORMAL_PRIORITY`
- Add `taskYIELD()` or `vTaskDelay(1)` after `start()` returns
- Investigate whether FreeRTOS threads at the same priority as the creating task get scheduled on nRF52