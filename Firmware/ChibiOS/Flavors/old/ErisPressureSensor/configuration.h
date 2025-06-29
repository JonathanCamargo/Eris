#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

#if defined(__MK20DX256__) || defined(__MK20DX128__) || \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define TEENSY
#elif defined(__SAM3X8E__) || defined(__SAM3X8H__)
#define ARDUINODUE
#else
#error unknown ARM processor
#endif

#define FIRMWARE_VERSION "v2.0" 

/* Global configuration such as pins, rates etc */

/***********************************************/
////////////////////////////////////////////////
// Analog pressure sensor
// Brand and part number*
//
// 
#define PIN_ANALOG A0 

////////////////////////////////////////////////
/// LED 
/// (classic arduino led)
/// 
#ifdef TEENSY
#define PIN_LED 13 //(Already defined in Arduino.h)
#endif

/////////////////////////////////////////////////
/// Failure LED
/// (LED that displays major error state)
/// 
#define PIN_ERROR 12 


/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 5 // Size for internal data buffers

// DEBUGGING FLAGS
//#define DEBUG_TIME false
//#define DEBUG_SYSCLK 180.0


// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


// Use SD Card to save data
//#define SDCARD true
#define SDBUFFERSIZE 200
#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD 0
#endif


//Enable Feature extraction
//#define FEATURES true


#endif
