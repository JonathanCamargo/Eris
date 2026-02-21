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
#ifndef PIN_LED
#define PIN_LED 13 //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 1
#define PIN_FSR A1
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

///////////////////////////////////////////////////
/// 
/// EMG CONFIGURATION
#define EMG_TXBUFFERSIZE 24 // Max samples to transmit in streaming
#define EMG_NUMCHANNELS 8
#define EMG_GAIN 2
#define CANTAPOK Can0

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 28

// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


// Enable SD Card
#define SDCARD true

#endif
