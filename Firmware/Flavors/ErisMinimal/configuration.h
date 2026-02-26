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

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 2
#define PIN_FSR_0 A0
#define PIN_FSR_1 A1
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change)
//
#define PIN_SYNC 2  // Pin 2 supports interrupts on all Arduinos
#define SYNC_TXBUFFERSIZE 24


// DEBUGGING FLAGS
#define DEBUG true

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisMinimal by ossip"

#endif
