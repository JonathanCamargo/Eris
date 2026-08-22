#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / ERIS_SERIAL_BAUD

#define FIRMWARE_VERSION "v3.0"

/***********************************************/
/* Global configuration such as pins, rates etc */
/***********************************************/

#if !defined(ESP32) && !defined(ARDUINO_ARCH_ESP32)
#error "ErisESP32 targets the ESP32 family. Select an ESP32 board, or use another flavor."
#endif

////////////////////////////////////////////////
/// LED
/// PIN_LED comes from eris_board.h (GPIO2, the DevKitC on-board LED). ESP32-S3
/// and C3 boards usually have an addressable LED instead -- #define PIN_LED to
/// a free GPIO before the eris_board.h include on those.
///
ERIS_PIN_ASSERT_OUTPUT(PIN_LED);

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define STRBUFFERSIZE 64

/////////////////////////////////////////////////////
/// SERIAL BAUD
/// ERIS_SERIAL_BAUD comes from eris_board.h (921600 here -- the practical
/// ceiling for a CP2102/CH340 bridge; the Python driver's 12 Mbps default only
/// works on Teensy, whose CDC link ignores baud). Pass a matching baudrate to
/// the eris.Eris() constructor on the host.

/////////////////////////////////////////////////////
/// CORE AFFINITY
/// Eris's buffers and critical sections assume a single-core model, so every
/// Eris thread is pinned to one core (see ERIS_ESP32_CORE in eris_rtos.h).
/// Core 1 is the core Arduino's own loopTask runs on; core 0 carries the WiFi
/// and BT stacks. Uncomment to move Eris to core 0 instead.
//#define ERIS_ESP32_CORE 0

// DEBUGGING FLAGS
#define DEBUG true
#define DEBUG_SYSCLK 240000000.0

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisESP32 by ossip"

#endif
