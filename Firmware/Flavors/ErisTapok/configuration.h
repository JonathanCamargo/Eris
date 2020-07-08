#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

#define FIRMWARE_VERSION "v2.0"

/* Global configuration such as pins, rates etc */

/***********************************************/
////////////////////////////////////////////////
// SPI ADC
// Brand and part number*
//
//

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

#define EMG_ADC_FREQUENCY_US 125 // 8kHz
#define FSR_ADC_FREQUENCY_US 2000 // 500Hz

//// SYNC signals
#define SYNC_FREQUENCY_US 5000000 // 0.2Hz
#define PIN_SYNC 28

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Size for transmision buffer last 85 points this should allways be smaller than erisbuffers MEMBUFFERSIZE
#define FSR_TXBUFFERSIZE 10 // To reduce bandwith transmit fewer FSR samples
#define EMG_TXBUFFERSIZE 24 
#define SYNC_TXBUFFERSIZE 10 //
#define STREAMING_PERIOD_MS 8 // With streaming period of 8ms @10kHz we get 80 samples per period




// DEBUGGING FLAGS
//#define DEBUG_TIME false
//#define DEBUG_SYSCLK 180.0
#define DEBUG_ADS1256

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

#define STRBUFFERSIZE 128

#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 28

// Impedance and temperature using Serial port
#define ETI_SERIAL Serial1

//Enable Feature extraction
#define FEATURES true


#endif
