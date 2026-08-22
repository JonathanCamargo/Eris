#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

// RTOS selection (nRF52 -> FreeRTOS) is handled automatically by eris_rtos.h.

#define FIRMWARE_VERSION "v3.0"

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// BOARD SELECTION
///
/// Pin numbers, the I2C bus and the serial baud differ per board, so they are
/// grouped per target below. Detection is automatic; to force a target, define
/// ERIS_BOARD_ESP32 or ERIS_BOARD_TEENSY before this header (or in build flags).
/// Any individual pin can also be overridden from outside -- each is #ifndef'd.
///
#if !defined(ERIS_BOARD_ESP32) && !defined(ERIS_BOARD_TEENSY) && !defined(ERIS_BOARD_OTHER)
  #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    #define ERIS_BOARD_ESP32 1
  #elif defined(TEENSYDUINO)
    #define ERIS_BOARD_TEENSY 1
  #else
    #define ERIS_BOARD_OTHER 1
  #endif
#endif

#if defined(ERIS_BOARD_ESP32)
  //// ---- ESP32 (DevKitC / WROOM-32 and clones) ----
  #ifndef PIN_LED
  #define PIN_LED 2            // on-board LED. The generic esp32 variant does
  #endif                       // not define LED_BUILTIN at all.
  #ifndef PIN_I2C_SDA
  #define PIN_I2C_SDA 21       // ESP32 default I2C pins. Any free GPIO works --
  #endif                       // the ESP32 I2C peripheral is routed via the GPIO
  #ifndef PIN_I2C_SCL          // matrix, so change these to match your wiring.
  #define PIN_I2C_SCL 22
  #endif
  #define ERIS_I2C_BEGIN()  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)
  #ifndef ERIS_SERIAL_BAUD
  #define ERIS_SERIAL_BAUD 921600   // UART-bridge ceiling; 12 Mbps is Teensy-only
  #endif

#elif defined(ERIS_BOARD_TEENSY)
  //// ---- Teensy 3.x / 4.x ----
  #ifndef PIN_LED
  #define PIN_LED 13           // classic Arduino LED
  #endif
  // Teensy pins the I2C bus in hardware, so there is nothing to pass to
  // Wire.begin(). To use an alternate bus, call Wire.setSDA()/setSCL() before
  // Servos::start(), or switch to Wire1/Wire2.
  #define ERIS_I2C_BEGIN()  Wire.begin()
  #ifndef ERIS_SERIAL_BAUD
  #define ERIS_SERIAL_BAUD 115200   // USB CDC ignores the value
  #endif

#else
  //// ---- Other boards (SAMD21, nRF52, ...) ----
  #ifndef PIN_LED
  #define PIN_LED LED_BUILTIN
  #endif
  #define ERIS_I2C_BEGIN()  Wire.begin()
  #ifndef ERIS_SERIAL_BAUD
  #define ERIS_SERIAL_BAUD 115200
  #endif
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

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisServo by ossip"

#endif
