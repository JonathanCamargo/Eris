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
#define PIN_LED 13

/////////////////////////////////////////////////
/// Failure LED
/// (LED that displays major error state)
///
#define PIN_ERROR 12

/////////////////////////////////////////////////
/// Analog inputs
/// EMG signals
///
// EMG CONFIGURATION
#define EMG_NUMCHANNELS 8
#define CANTAPOK Can0

/// FSR signals
#define FSR_NUMCHANNELS 1
#define PIN_FSR A2

#define FSR_ADC_FREQUENCY_HZ 100
#define FSR_ADC_PERIOD_MS ((1.0/FSR_ADC_FREQUENCY_HZ)*1000)

//// SYNC signals
#define SYNC_FREQUENCY_US 5000000 // 0.2Hz
#define PIN_SYNC 28

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define FSR_TXBUFFERSIZE 10 // To reduce bandwith transmit fewer FSR samples
#define EMG_TXBUFFERSIZE 24 
#define SYNC_TXBUFFERSIZE 10 //
#define STREAMING_PERIOD_MS 10 // Period of

// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0


// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

#endif
