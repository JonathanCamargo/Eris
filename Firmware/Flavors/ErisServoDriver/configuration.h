#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

// RTOS selection (nRF52 -> FreeRTOS) is handled automatically by eris_rtos.h.

#define FIRMWARE_VERSION "v3.0"

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
/// SERVO CONFIGURATION (PCA9685 via I2C)
///
#define NUM_SERVOS 16
#define SERVO_MIN_PULSE 150   // ~0 degrees (Adafruit PWM ticks out of 4096)
#define SERVO_MAX_PULSE 600   // ~180 degrees
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define PCA9685_I2C_ADDR 0x40 // Default PCA9685 address

// Smooth movement (rate-limited stepping with ease-out)
#define SERVO_SMOOTH_SPEED     180.0  // Cruise speed (deg/sec)
#define SERVO_SMOOTH_DECEL_DEG 6.0   // Degrees from target where deceleration begins
#define SERVO_SMOOTH_LOOP_MS   5     // Tick interval (200 Hz)

// I2C bus speed for the PCA9685 (fast-mode; needed so a full NUM_SERVOS
// update fits inside one smooth/demo tick).
#define SERVO_I2C_CLOCK_HZ     400000

// DEMO mode: sine-oscillate all servos (0..NUM_SERVOS-1) with a phase offset
// per channel so they chase each other in a traveling wave. The demo sweeps
// within its own sub-range (need not be the full SERVO_MIN/MAX_ANGLE travel).
#define DEMO_MIN_ANGLE   70.0f     // demo lower bound (deg)
#define DEMO_MAX_ANGLE   110.0f    // demo upper bound (deg)
#define DEMO_FREQ_HZ     0.5f      // one full cycle / 2 s


// DEBUGGING FLAGS
#define DEBUG true

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisServo by ossip"

#endif
