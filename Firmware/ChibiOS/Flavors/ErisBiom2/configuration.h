#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

#if defined(__MK20DX256__) || defined(__MK20DX128__) || \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define TEENSY
#elif defined(__SAM3X8E__) || defined(__SAM3X8H__)
#define ARDUINODUE
#else
#define TEENSY
//#error unknown ARM processor
#endif

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
// BIOMETRICS CONFIGURATION
//
#define BIOM_TXBUFFERSIZE 16
#define BIOM_HIP_GON_CHAN 0
#define BIOM_NUMCHANNELS 6
#define PIN_BIOM A0, A1, A2, A3, A4, A5
#define BIOM_FREQUENCY_HZ 1000
#define BIOM_PERIOD_US ((1.0/BIOM_FREQUENCY_HZ)*1000000)

////////////////////////////////////////////////
/// FEATURE SETTINGS
///
///
#define FEATS_NUM 9
#define FEATS_NUMCLASSLOCATIONS 6 //number of gait locations we classify at

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 29
#define SYNC_TXBUFFERSIZE 24

// DEBUGGING FLAGS
#define DEBUG false
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

// BIOM ADC CONFIGURATION
#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 28



#endif
