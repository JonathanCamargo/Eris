#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

// On nRF52 boards, the core always links FreeRTOS. Using ChRt causes
// SVC_Handler conflicts with the built-in FreeRTOS port.
#if defined(NRF5) || defined(NRF52840_XXAA) || defined(NRF52)
#define ERIS_USE_FREERTOS
#endif

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
/// SERVO CONFIGURATION (Arduino Servo.h, direct PWM)
///
// NUM_SERVOS must match the length of SERVO_PINS. Limited by available
// hardware PWM channels (~12 on Teensy 4.x, fewer on SAMD21 / AVR).
#define NUM_SERVOS 6
#define SERVO_PINS 2, 3, 4, 5, 6, 7
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Smooth movement (rate-limited stepping with ease-out)
#define SERVO_SMOOTH_SPEED     180.0  // Cruise speed (deg/sec)
#define SERVO_SMOOTH_DECEL_DEG 6.0   // Degrees from target where deceleration begins
#define SERVO_SMOOTH_LOOP_MS   5     // Tick interval (200 Hz)


// DEBUGGING FLAGS
#define DEBUG true

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisServo by ossip"

#endif
