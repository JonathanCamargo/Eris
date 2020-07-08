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
// ANALOG READINGS
//
//
// 
#define PIN_ANALOG A0 
#define PIN_CH0 A0
#define PIN_CH1 A1
#define NUMEMGCHANNELS 8
////////////////////////////////////////////////
// IMU READINGS
#define PIN_SS0 8
#define NUMIMUCHANNELS 1
#define IMU_FREQUENCY_US 10000 // 100Hz
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

/////////////////////////////////////////////////
/// Analog inputs
/// (LED that displays major error state)
/// 


#define ADC_FREQUENCY_US 1000 // 1kHz



/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 10 // Size for transmision buffer last 5 points
#define STRBUFFERSIZE 128

// DEBUGGING FLAGS
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0
#define DEBUG_ADS1256

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

// EMG ADC CONFIGURATION
#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 28

//Enable Feature extraction
#define FEATURES true
#define FEATURESSIZE 5

//Interrupt pin to start / stop streaming
#define STREAM_INT_PIN digitalPinToInterrupt(0) 

#endif
