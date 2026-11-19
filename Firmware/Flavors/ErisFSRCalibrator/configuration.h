#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>  // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0"
#define MAXSTRCMP 8  // Maximum length for string comparison

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED
/// (classic arduino led)
///
#ifndef PIN_LED
#define PIN_LED 13  //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16  // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// A4988 STEP/DIR/EN pins
///
/// Plain GPIO, any free digital pin works. EN is active LOW on the A4988:
/// LOW energizes the outputs, HIGH releases the motor. On most breakouts
/// (Pololu and clones) EN has a pull-down, so an unconnected EN means
/// "always energized" and EN/ZERO/release then only affect the bookkeeping.
///
/// Microstepping is set in HARDWARE by MS1/MS2/MS3 on the A4988. The firmware
/// cannot read or change it, so A4988_MICROSTEPS below must match your jumpers
/// or every distance will be off by the ratio.
///   MS1 MS2 MS3:  L L L = 1   H L L = 2   L H L = 4   H H L = 8   H H H = 16
///
/// RESET and SLEEP must be tied together (or both pulled high) or the A4988
/// does nothing at all. On the Pololu board they are adjacent pins; jumper them.
///
/// The ESP32 map differs because GPIO 2/3 are the strapping pin and the USB
/// console RX. motion.cpp runs ERIS_PIN_ASSERT_OUTPUT on all three, which
/// catches flash and input-only pins at compile time.
#ifndef PIN_A4988_STEP
#if defined(ERIS_BOARD_ESP32S3)
// XIAO ESP32S3 silk: D8 / D9 / D10. The classic-ESP32 25/26/27 below would be
// fatal here: 26/27 are the S3's SPI flash pins (TG1WDT reset loop).
#define PIN_A4988_STEP 7
#define PIN_A4988_DIR 8
#define PIN_A4988_EN 9
#elif defined(ERIS_BOARD_ESP32)
#define PIN_A4988_STEP 25
#define PIN_A4988_DIR 26
#define PIN_A4988_EN 27
#else
#define PIN_A4988_STEP 2
#define PIN_A4988_DIR 3
#define PIN_A4988_EN 4
#endif
#endif

// Flips the sense of DIR without rewiring the motor.
#define STEPPER_INVERT_DIR 0

/// Must match MS1/MS2/MS3 (see above).
#define A4988_MICROSTEPS 16

////////////////////////////////////////////////
/// Motion / axis geometry
///
/// Steps per millimetre of travel: full steps/rev x microsteps / mm per rev.
/// Default: a 1.8 deg motor on an 8 mm lead screw, 16 microsteps -> 400 steps/mm.
/// A 2 mm lead (T8x2) screw would be 1600; a direct-drive M5 thread (0.8 mm) 4000.
#define STEPPER_FULL_STEPS_PER_REV 200
#define STEPPER_MM_PER_REV 8.0f
#define STEPPER_STEPS_PER_MM \
  ((float)STEPPER_FULL_STEPS_PER_REV * A4988_MICROSTEPS / STEPPER_MM_PER_REV)

/// Motion tick. Step pulses for one tick are emitted as a burst at the top of
/// the tick, so the RTOS stays in charge of the CPU -- see motion.cpp.
#define STEPPER_TICK_MS 1
/// STEP pulse half-period. The A4988 needs 1 us high and 1 us low minimum;
/// 3 us leaves margin for slow edges and level shifters.
#define STEPPER_STEP_PULSE_US 3
/// Ceiling on steps per tick = top step rate in kHz (16 -> 16 kHz -> 40 mm/s
/// at 400 steps/mm). Also bounds how long the motion thread holds the CPU.
#define STEPPER_MAX_STEPS_PER_TICK 16

/// Feedrates are G-code feedrates: millimetres per MINUTE.
/// Calibration is slow by nature: you want the FSR and the load cell to settle
/// at every point, and you do not want to overshoot into the load cell.
#define STEPPER_DEFAULT_FEED 60.0f  // 1 mm/s
#define STEPPER_MAX_FEED 600.0f     // 10 mm/s
#define STEPPER_MIN_FEED 1.0f
/// Acceleration in mm/s^2 (trapezoidal ramp on every move).
#define STEPPER_ACCEL 50.0f

/// Soft limits in mm, applied to every G0 target. No homing: "0" is wherever
/// the axis was at power-up or at the last ZERO. Set STEPPER_SOFT_LIMITS 0 to
/// accept any target.
#define STEPPER_SOFT_LIMITS 1
#define STEPPER_MIN_POS_MM -50.0f
#define STEPPER_MAX_POS_MM 50.0f

////////////////////////////////////////////////
/// HX711 load cell amplifier
///
/// Two wires, neither of them a bus: DOUT (data out, also "data ready" -- it
/// goes LOW when a conversion is waiting) and PD_SCK (clock, also power-down).
/// Any two digital pins work.
///
/// Conversion rate is set in HARDWARE by the HX711's RATE pin:
///   RATE = GND -> 10 SPS (most green breakouts are hard-wired here)
///   RATE = VCC -> 80 SPS (SparkFun board: cut/bridge the jumper)
/// LOADCELL_SPS must say which, only so the startup banner and the tare time
/// are right -- the firmware follows whatever the chip produces.
#ifndef PIN_HX711_DOUT
#if defined(ERIS_BOARD_ESP32S3)
// XIAO ESP32S3 silk: D4 / D5. (19 is the S3's native USB D-.)
#define PIN_HX711_DOUT 5
#define PIN_HX711_SCK 6
#elif defined(ERIS_BOARD_ESP32)
#define PIN_HX711_DOUT 18
#define PIN_HX711_SCK 19
#else
#define PIN_HX711_DOUT 5
#define PIN_HX711_SCK 6
#endif
#endif

#define LOADCELL_SPS 10
/// 128 = channel A, gain 128 (the usual load-cell input, +-20 mV full scale).
/// 64 = channel A gain 64; 32 = channel B gain 32.
#define LOADCELL_GAIN 128

/// Counts per newton, i.e. what LCAL measures. Run LCAL once with a known
/// weight, read back the value it prints, and paste it here so the next boot
/// is calibrated. Leave LOADCELL_CALIBRATED at 0 until then: force is then
/// reported in raw counts (tared) and the overload guard is OFF, because a
/// limit in newtons means nothing without a scale.
#define LOADCELL_COUNTS_PER_N 1.0f
#define LOADCELL_CALIBRATED 0

/// Readings averaged by TARE and LCAL. At 10 SPS, 20 samples = 2 s.
#define LOADCELL_AVG_SAMPLES 20

/// The load cell is considered absent if DOUT has not gone low for this long.
/// The HX711 wakes in ~400 ms at 10 SPS, so stay well above that.
#define LOADCELL_TIMEOUT_MS 1000

////////////////////////////////////////////////
/// Overload guard
///
/// The stepper is pushing on the load cell (through the FSR). A lead screw
/// produces far more force than a small load cell is rated for, and nothing
/// else in the loop knows when to stop. When |force| exceeds CAL_FORCE_LIMIT_N
/// the axis halts immediately and any further move in the pressing direction is
/// refused until the force falls below CAL_FORCE_RELEASE * limit.
///
/// Set it below your load cell's rated capacity AND below what the FSR can
/// take. Only active when the load cell is calibrated (see above).
#define CAL_FORCE_LIMIT_N 50.0f
#define CAL_FORCE_RELEASE 0.9f
/// Which way presses: +1 if increasing X moves the probe INTO the FSR,
/// -1 if decreasing X does.
#define CAL_PRESS_DIR 1

////////////////////////////////////////////////
/// FSR voltage divider
///
///     VREF --[ FSR ]--+--[ R_FIXED ]-- GND
///                     |
///                  ERIS_ADC(0)
///
/// In this orientation the voltage RISES with force:
///     V = VREF * R_FIXED / (R_FSR + R_FIXED)
/// so R_FSR = R_FIXED * (VREF - V) / V. The firmware streams V; resistance and
/// conductance are a host-side calculation from these constants.
/// Pick R_FIXED near the FSR's resistance in the middle of the force range you
/// care about (10k is the usual starting point for an Interlink 402).
///
/// Power the divider from the board's ANALOG reference (3V3 on every Eris
/// board), never from 5 V -- the ADC pin would see more than it can take.
/// ERIS_ADC(0) = GPIO 36 on a classic ESP32, GPIO 1 (A0 / D0) on a XIAO ESP32S3.
#define PIN_FSR ERIS_ADC(0)
#define FSR_VREF 3.3f
#define FSR_R_FIXED 10000.0f  // ohms, documented in the stream's README
#define FSR_ADC_BITS 12       // Teensy/SAMD/nRF52/ESP32 all do 12 bits
/// analogRead()s averaged per sample. An FSR is slow; noise is not.
#define FSR_OVERSAMPLE 8

////////////////////////////////////////////////
/// Calibration sample stream
///
/// Polled at CAL_POLL_HZ. A sample is emitted only on ticks where the HX711
/// has a fresh conversion, and the FSR is read on that same tick -- so every
/// row of the stream is a matched (voltage, force) pair, which is what a
/// calibration fit needs. The stream rate is therefore the HX711's rate
/// (10 or 80 Hz), not CAL_POLL_HZ.
///
/// The poll rate bounds the timing error between the two readings: at 200 Hz
/// the HX711 conversion is at most 5 ms old when it is paired with the FSR.
/// Must divide 1000 evenly (ERIS_SENSOR static_asserts it).
///
/// If the load cell is absent the FSR still streams, at CAL_FALLBACK_HZ,
/// with fresh=0 and force=0, so the divider can be checked on its own.
#define CAL_POLL_HZ 200
#define CAL_FALLBACK_HZ 20
#define CAL_TXBUFFERSIZE 10

// DEBUGGING FLAGS
#define DEBUG true

#define FIRMWARE_INFO ERIS_FIRMWARE_INFO("ErisFSRCalibrator")

#endif
