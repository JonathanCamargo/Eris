#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

/*
#if defined(__MK20DX256__) || defined(__MK20DX128__) || \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define TEENSY
#elif defined(__SAM3X8E__) || defined(__SAM3X8H__)
#define ARDUINODUE
#else
#error unknown ARM processor
#endif*/
#define TEENSY

#define FIRMWARE_VERSION "v3.0" 

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED 
/// (classic arduino led)
/// 
#ifdef TEENSY
#define PIN_LED 14
#endif

/////////////////////////////////////////////////
/// Failure LED
/// (LED that displays major error state)
/// 
#define PIN_ERROR 12 


/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 48 // Size for transmision buffer last 24 points
#define STRBUFFERSIZE 128


// DEBUGGING FLAGS
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0
#define DEBUG_ADS1256

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

//////////////////////////////////////////////////
/// 
/// EMG CONFIGURATION
#define EMG_TXBUFFERSIZE 24 // Max samples to transmit in streaming
#define EMG_NUMCHANNELS 8
#define PIN_EMG_ADC0_SS 8
#define PIN_EMG_ADC0_DRDY 7
#define PIN_EMG_RESET 6

////////////////////////////////////////////////
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 1
#define PIN_FSR A13
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)


/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 28

#endif
