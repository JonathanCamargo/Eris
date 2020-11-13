#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

#define FIRMWARE_VERSION "v2.0" 

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED 
/// (classic arduino led)
/// 
#ifdef TEENSY
#define PIN_LED 13 //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define ANALOG_TXBUFFERSIZE 16
#define TXBUFFERSIZE 16 // Size for transmision buffer last 16 points

#define STRBUFFERSIZE 128


///////////////////////////////////////////////////
/// EMG ANALOG CONFIGURATION
#define ANALOG_NUMCHANNELS 8
#define ANALOG_PERIOD_US 1000
#define PINS_ANALOG A0,A1,A2,A3,A4,A5,A6,A7

/// FSR ANALOG CONFIGURATION
#define FSR_NUMCHANNELS 1
#define FSR_PERIOD_US 10000 // FSR @100Hz
#define PIN_FSR A20

////////////////////////////////////////////////////
/// Serial inputs
/// Impedance and temperature using Serial port
#define ETI_NUMCHANNELS 1
#define ETI_SERIAL0 Serial2

// DEBUGGING FLAGS
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "NFEris by ossip"

#define PIN_LED 13

#endif
