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
// SPI ADC
// Brand and part number*
//
// 
#define PIN_ANALOG A0 // We do not have SPI YET Will can make this? 

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


////////////////////////////////////////////////
///
/// ELMO DRIVER
#define ELMO_CANID0 126 // Knee Elmo
#define ELMO_CANID1 127 // Ankle Elmo

#define ELMO_MINCURRENT -15.0  // (A)
#define ELMO_MAXCURRENT 15.0   // (A)
#define ELMO_KTCONSTANT 0.038     // (Nm/A)
#define ELMO_RATE_US 4000      // Elmo update rate in microseconds
#define ELMO_MAX_COUNTS_INCREMENTAL 16384    // Maximum counts for the incremental encoder

////////////////////////////////////////////////
///
/// SRI LOADCELL DRIVER
#define SRILOADCELL_CANID 128  //Foot loadcell
#define SRILOADCELL_RATE_US 10000      // Elmo update rate in microseconds
////////////////////////////////////////////////

////////////////////////////////////////////////
///
/// MECHANICAL TRANSMISION
#define KNEE_GEAR_RATIO 120
#define ANKLE_GEAR_RATIO 175
/***********************************************/


/////////// MEMORY CONFIGURATION
#define MEMBUFFERSIZE 40 // Size for internal data buffers
#define ERRORBUFFERSIZE 10 // Size for internal data buffers
#define TXBUFFERSIZE 5 // Size for transmision buffer last 5 points


// DEBUGGING FLAGS
//#define DEBUG_TIME false
//#define DEBUG_SYSCLK 180.0


// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


// Use SD Card to save data
#define SDCARD true
#define SDBUFFERSIZE 200
#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD 0
#endif


#endif
