#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

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
#define SERVO_SMOOTH_MAX_STEP 0.45   // Max degrees per tick (0.45 * 200Hz = 90 deg/sec)
#define SERVO_SMOOTH_LOOP_MS 5       // Tick interval (200 Hz)
#define SERVO_SMOOTH_GAIN    0.08    // Proportional gain for ease-out (decel zone ~5.6 deg)
#define SERVO_SMOOTH_EPSILON 0.02    // Snap-to-target threshold (degrees)


// DEBUGGING FLAGS
#define DEBUG true

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisServo by ossip"

#endif
