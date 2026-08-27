#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0" 
#define MAXSTRCMP 5 // Maximum length for string comparison

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED 
/// (classic arduino led)
/// 
#ifndef PIN_LED
#define PIN_LED 13   //(Already defined in Arduino.h)
#endif

#define PIN_LED_R 28 //

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

///////////////////////////////////////////////////
///
///  IMU CONFIGURATION
#define IMU_TXBUFFERSIZE 10
#define IMU_COUNT 2            // MPU9250s on the bus: 0x68 and 0x69
// Integer Hz, and it must divide 1000 evenly -- ERIS_SENSOR_MULTI static_asserts
// this, because the period has to land on a whole scheduler tick. 250 -> 4 ms.
#define IMU_FREQUENCY_HZ 250

// DEBUGGING FLAGS
#define DEBUG true

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

#endif
