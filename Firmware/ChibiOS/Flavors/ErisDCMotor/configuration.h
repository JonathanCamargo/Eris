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
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define STRBUFFERSIZE 64



////////////////////////////////////////////////
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 2
#define PIN_FSR_0 A0
#define PIN_FSR_1 A1
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_MS ((1.0/FSR_FREQUENCY_HZ)*1000)


////////////////////////////////////////////////
/// POT analog input
///
#define POT_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define POT_NUMCHANNELS 1
#define PIN_POT_0 A9
#define PIN_POT_1 A1
#define POT_FREQUENCY_HZ 100
#define POT_PERIOD_MS ((1.0/POT_FREQUENCY_HZ)*1000)

////////////////////////////////////////////////
/// Motor pins
///
#define PIN_MOT_0_A 0
#define PIN_MOT_0_B 1

/////////////////////////////////////////////////////
/// STREAMING
#define STREAMING_PERIOD_MS 10

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 29
#define SYNC_TXBUFFERSIZE 24


// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


#endif
