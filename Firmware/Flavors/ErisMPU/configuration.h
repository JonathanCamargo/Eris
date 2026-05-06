#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

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
#define IMU_FREQUENCY_HZ 250.0
#define IMU_PERIOD_US ((1.0/IMU_FREQUENCY_HZ)*1000000)

// DEBUGGING FLAGS
#define DEBUG true

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

#endif
