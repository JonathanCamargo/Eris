#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0" 

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED 
#define PIN_LED 13/// (classic arduino led)
/// 
#define NUMCHANNELS 10
/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define STRBUFFERSIZE 64

// DEBUGGING FLAGS
#define DEBUG false
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


#endif
