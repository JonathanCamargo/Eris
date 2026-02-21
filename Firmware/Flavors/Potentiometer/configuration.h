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

#define PIN_LED_R 28 //

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// POTENTIOMETER analog input
///
#define POTENTIOMETER_GAIN 270.0/3.3 // deg/V
#define POTENTIOMETER_OFFSET 0 // deg
#define POTENTIOMETER_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define PIN_POTENTIOMETER A0
#define POTENTIOMETER_FREQUENCY_HZ 100
#define POTENTIOMETER_PERIOD_MS ((1.0/POTENTIOMETER_FREQUENCY_HZ)*1000)

// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


#endif
