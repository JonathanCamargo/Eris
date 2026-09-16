#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0"
#define MAXSTRCMP 8 // Maximum length for string comparison

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED
/// (classic arduino led)
///
#ifndef PIN_LED
#define PIN_LED 13   //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// TMC2209 STEP/DIR pins
///
/// These three are plain GPIO -- any free digital pin works. EN is active LOW
/// on every TMC2209 breakout (BigTreeTech, Watterott, FYSETC): LOW energizes
/// the motor, HIGH releases it. Leaving it floating also releases the motor,
/// so nothing moves until the firmware drives it.
///
/// The ESP32 map differs because GPIO 3 is the USB console RX and GPIO 2 is a
/// strapping pin -- both compile and upload fine and then take the serial link
/// down with them. motion.cpp runs ERIS_PIN_ASSERT_OUTPUT on all three, which
/// catches the flash and input-only pins at compile time.
#ifndef PIN_TMC_STEP
#if defined(ERIS_BOARD_ESP32)
#define PIN_TMC_STEP 25
#define PIN_TMC_DIR  26
#define PIN_TMC_EN   27
#else
#define PIN_TMC_STEP 2
#define PIN_TMC_DIR  3
#define PIN_TMC_EN   4
#endif
#endif

// Flips the sense of DIR without rewiring the motor.
#define STEPPER_INVERT_DIR 0

////////////////////////////////////////////////
/// TMC2209 UART (configuration only -- no motion goes over this link)
///
/// Wiring is single-wire half duplex: the driver has ONE pin (PDN_UART) for
/// both directions.
///
///     MCU TX --[1k]--> PDN_UART
///     MCU RX ----------+
///
/// The series resistor is what lets the driver talk back over the same wire
/// while the MCU's TX idles high. It also means the MCU hears its own
/// transmission echoed back -- tmc2209.cpp discards that echo rather than
/// mistaking it for a reply.
///
/// TMC_ADDRESS is set by the MS1/MS2 pins on the board: 00=0, 01=1, 10=2,
/// 11=3. Most single-driver breakouts tie both low -> address 0.
#ifndef TMC_UART
#if defined(ERIS_BOARD_ESP32)
#define TMC_UART Serial2          // Serial1's default pins collide with flash
#define PIN_TMC_UART_RX 16
#define PIN_TMC_UART_TX 17
#else
#define TMC_UART Serial1
#endif
#endif

/// How the port is opened. ESP32 routes UARTs through the GPIO matrix, so it
/// needs the pins passed in; everywhere else the pins are fixed in silicon.
#ifndef ERIS_TMC_UART_BEGIN
#if defined(ERIS_BOARD_ESP32)
#define ERIS_TMC_UART_BEGIN()  TMC_UART.begin(TMC_UART_BAUD, SERIAL_8N1,                                               PIN_TMC_UART_RX, PIN_TMC_UART_TX)
#else
#define ERIS_TMC_UART_BEGIN()  TMC_UART.begin(TMC_UART_BAUD)
#endif
#endif

#define TMC_UART_BAUD    115200
#define TMC_ADDRESS      0
// Timeout for one read datagram. A TMC2209 answers in well under a ms at
// 115200 baud; anything longer means the wire, the resistor or the address
// is wrong.
#define TMC_REPLY_TIMEOUT_MS 20
// How long the UART code busy-waits between polls of the receive register. It
// busy-waits rather than sleeping because TMC::begin() runs before the FreeRTOS
// scheduler is started -- see the comment on tmcWaitByte() in tmc2209.cpp.
#define TMC_POLL_US 50

/// Sense resistor, in ohms. This is a property of YOUR breakout and it is the
/// single most important number here: it scales the current setting, so a
/// wrong value silently runs the motor hot or weak.
///   BigTreeTech TMC2209 v1.2 / v1.3, FYSETC: 0.11
///   Watterott SilentStepStick:             0.11
///   Some clones:                           0.15
#define TMC_RSENSE 0.11f

/// Motor RMS current in mA. Start low. A NEMA17 rated 1.5 A/phase wants about
/// 1000 mA RMS with adequate cooling; 600 mA is a safe first run.
#define TMC_RMS_CURRENT_MA 600
/// Hold current as a fraction of run current, applied after the motor has been
/// idle for TMC_POWERDOWN_DELAY ticks.
#define TMC_HOLD_MULTIPLIER 0.5f
#define TMC_POWERDOWN_DELAY 10

/// Microsteps per full step: 1,2,4,8,16,32,64,128,256. Set over UART, which
/// overrides whatever MS1/MS2 select in standalone mode.
#define TMC_MICROSTEPS 16

/// StealthChop (quiet, lower torque at speed) vs SpreadCycle (louder, better
/// high-speed torque). 1 = StealthChop.
/// Leave this at 1 if you want stall detection: StallGuard4 on the TMC2209 is
/// specified for StealthChop, and SpreadCycle readings are too noisy to
/// threshold.
#define TMC_STEALTHCHOP 1

////////////////////////////////////////////////
/// HARD STOP / STALL DETECTION (StallGuard4)
///
/// What this is NOT: a current threshold. A chopper driver regulates coil
/// current to a constant, so running the axis into a hard stop does not make
/// the current rise -- it was already at IRUN. Watching current would detect
/// nothing.
///
/// What the TMC2209 gives you instead is StallGuard4: the driver measures the
/// motor's load angle from its own back-EMF and reports SG_RESULT, 0..1023.
/// SG_RESULT is HIGH when the motor is unloaded and FALLS as mechanical load
/// rises, reaching ~0 at the point the rotor is about to lose sync. That is
/// the threshold to set, and it is the same physical quantity a current
/// threshold would be reaching for -- read from the right place.
///
///   stall  <=>  SG_RESULT <= 2 * TMC_SGTHRS
///
/// TMC_SGTHRS is therefore INVERTED relative to intuition: a HIGHER value
/// trips EARLIER (more sensitive). It has to be tuned per machine, per
/// current, per speed -- there is no universal value. Stream the axis and
/// watch the `sg` trace while the motor runs free, then set TMC_SGTHRS to a
/// bit under half the unloaded reading. `SG <n>` retunes it live.
#define STEPPER_STALL_DETECT 1
#define TMC_SGTHRS 60

/// Enables StallGuard/DIAG over the whole usable speed range (the register is
/// a step-period threshold, so a large value means "down to a low speed").
#define TMC_TCOOLTHRS 0xFFFFFUL

/// The DIAG pin, if you wire it: the driver pulls it HIGH the instant
/// SG_RESULT crosses the threshold, so the motion thread catches the stall on
/// the very next tick with a digitalRead and no UART traffic at all. Without
/// it the same comparison is done by polling SG_RESULT over UART from the
/// telemetry thread, which costs one transaction every STEPPER_SG_DIVIDER
/// samples and detects a hard stop that much later.
/// DIAG also goes high on a driver fault (overtemperature, short), which is
/// why the report path reads DRV_STATUS to say which it was.
#define TMC_USE_DIAG 0
#define PIN_TMC_DIAG 5

/// SG_RESULT is meaningless below a certain speed (too little back-EMF) and
/// during acceleration (the load angle is still settling). Both gates have to
/// pass before a stall is believed, or the axis trips on every move it starts.
#define STEPPER_STALL_MIN_FEED 60.0f   // mm/min
#define STEPPER_STALL_ARM_MS   150     // after the move starts

/// 1 = halt on stall (what you want for crash / hard-stop detection),
/// 0 = report it in the stream and keep going.
#define STEPPER_STALL_HALT 1

/// Telemetry samples between SG_RESULT reads. Every read is a UART round trip,
/// so this is also the stall-polling rate when DIAG is not wired:
/// 2 -> 50 Hz at STEPPER_FREQUENCY_HZ 100, i.e. up to 20 ms of travel past the
/// hard stop before it is noticed.
#define STEPPER_SG_DIVIDER 2

////////////////////////////////////////////////
/// Motion / axis geometry
///
/// Steps per millimetre of travel. Full steps per rev x microsteps / mm per rev.
/// The default is the classic 3D-printer X axis: 200 * 16 / (20 teeth * 2 mm) = 80.
/// A leadscrew axis with 8 mm lead would be 200 * 16 / 8 = 400.
#define STEPPER_STEPS_PER_MM 80.0f

/// Motion tick. The step pulses for one tick are emitted as a burst at the top
/// of the tick, so the RTOS scheduler stays in charge of the CPU -- see the
/// comment in motion.cpp. 1 ms is one scheduler tick on every supported board.
#define STEPPER_TICK_MS 1
/// STEP pulse half-period. The TMC2209 needs 100 ns minimum; 3 us is generous
/// and survives the level shifting on 5 V boards.
#define STEPPER_STEP_PULSE_US 3
/// Ceiling on steps emitted in one tick, i.e. the top step rate in steps/s
/// (32 -> 32 kHz -> 400 mm/s at 80 steps/mm). It also bounds how long the
/// motion thread can hold the CPU: 32 * 2 * 3 us = ~200 us of a 1 ms tick.
#define STEPPER_MAX_STEPS_PER_TICK 32

/// Feedrates are G-code feedrates: millimetres per MINUTE.
#define STEPPER_DEFAULT_FEED 600.0f
#define STEPPER_MAX_FEED     3000.0f
#define STEPPER_MIN_FEED     6.0f
/// Acceleration in mm/s^2. Applied as a trapezoidal ramp on every move; a
/// stepper commanded straight to speed from standstill just stalls.
#define STEPPER_ACCEL 500.0f

/// Soft limits in mm, applied to every G0 target. There is no homing in this
/// flavor, so "0" is wherever the axis was at power-up (or at the last ZERO).
/// Set STEPPER_SOFT_LIMITS to 0 to accept any target.
#define STEPPER_SOFT_LIMITS 1
#define STEPPER_MIN_POS_MM -200.0f
#define STEPPER_MAX_POS_MM  200.0f

////////////////////////////////////////////////
/// Telemetry
///
/// Rate the axis state is sampled into the stream. Must divide 1000 evenly
/// (ERIS_SENSOR static_asserts it).
#define STEPPER_FREQUENCY_HZ 100
#define STEPPER_TXBUFFERSIZE 10
/// Read DRV_STATUS off the driver once every N telemetry samples. A UART
/// transaction takes about a millisecond, so it does not belong on every tick.
#define STEPPER_STATUS_DIVIDER 10

// DEBUGGING FLAGS
#define DEBUG true

#define FIRMWARE_INFO ERIS_FIRMWARE_INFO("ErisStepper")

#endif
