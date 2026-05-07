# ErisBici

Bicycle pedal-assist controller. Generates a PWM signal whose frequency toggles between two speeds based on a single button input (press to toggle on/off, hold to change speed). Not a streaming flavor — runs as a closed-box embedded controller.

## Status
Experimental (Arduino Nano-style standalone controller)

## Hardware
- **Target board:** Arduino-compatible board with attachInterrupt support on `PIN_BTN` (originally Arduino Nano per the project README's "Bicycle interface (Arduino Nano)" entry)
- **RTOS:** Currently uses raw ChibiOS event APIs (`chEvtRegisterMaskWithFlags`, `chEvtBroadcastFlagsI`) — does not run on FreeRTOS
- **Sensor / interface:** Single momentary button; PWM output on `PIN_LED`

## Pin assignments
- `PIN_LED 13` (PWM output to motor controller / LED)
- `PIN_BTN 2` (button input, INPUT_PULLUP, change-interrupt)
- `FREQ_LOW_HZ 10`, `FREQ_HIGH_HZ 45` (two assist speeds)
- `TIME_MIN_HOLD_MS 800` (button-hold threshold)

## Dependencies
- eriscommon
- ChRt (ChibiOS) — uses event APIs not abstracted by `eris_rtos.h`
- *No* SerialCommand / PacketSerial / streaming modules

## Serial Commands
None registered. The flavor prints state transitions (`press`, `release`, `Toggle ON/OFF`, `Change speed`) over Serial for debugging only.

## Sample Session
Power on the board with `PIN_BTN` wired to a momentary switch. Press the switch to start PWM at the current speed; press again to stop. Hold for >800 ms to switch between low and high speed.

## Notes
- This flavor predates the `eris_rtos.h` abstraction layer and uses ChibiOS event objects directly. Porting to FreeRTOS would require replacing the event-source/listener pattern.
- No `streaming.cpp` or `serialcommand.cpp` — this is a one-of-a-kind embedded controller, not a DAQ flavor.
