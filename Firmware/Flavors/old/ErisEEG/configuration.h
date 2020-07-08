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
/// LED 
/// (classic arduino led)
/// 
#ifdef TEENSY
#define PIN_LED 14
#endif

// FSR signals
#define PIN_FSR A2
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 48 // Size for transmision buffer last 24 points
#define SDBUFFERSIZE 512
#define FSR_TXBUFFERSIZE 10 // To reduce bandwith transmit fewer FSR samples
#define STREAMING_PERIOD_MS 8 // With streaming period of 8ms @10kHz we get 80 samples per period
#define STRBUFFERSIZE 128

// DEBUGGING FLAGS
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0
#define DEBUG_ADS1256

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

// EMG ADC CONFIGURATION
#define NUMEEGCHANNELS 2
#define PIN_ADC0_SS 8
#define PIN_ADC0_DRDY 7


#endif
