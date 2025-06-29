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
#define PIN_LED 13 //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define FSR_TXBUFFERSIZE 10
#define ETI_TXBUFFERSIZE 2
#define TXBUFFERSIZE 16 // Size for transmision buffer last 16 points
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define SDBUFFERSIZE_ETI 1 // Size of storage buffer for ETI (is less as it is collected at very low freq)
#define STRBUFFERSIZE 128

/////////////////////////////////////////////////
/// ADS131 Interface
///
///
#define EMG_NUMCHANNELS 3
#define PIN_ADC0_GPIO4 6
#define PIN_ADC0_GPIO3 7
#define PIN_ADC0_GPIO2 8
#define PIN_ADC0_GPIO1 32
#define PIN_ADC0_DRDY 5
#define PIN_ADC0_SS 10
#define PIN_ADC0_START 24
#define PIN_ADC0_RESET 25
#define PIN_ADC0_PWDN 26
#define PIN_ADC0_TESTN 27
#define PIN_ADC0_TESTP 28

// DEBUGGING FLAGS
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisADS131 by ossip and wf"

#endif
