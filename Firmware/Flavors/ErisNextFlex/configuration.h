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
#define EMG_TXBUFFERSIZE 10
#define FSR_TXBUFFERSIZE 10
#define ETI_TXBUFFERSIZE 2
#define TXBUFFERSIZE 16 // Size for transmision buffer last 16 points
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define SDBUFFERSIZE_ETI 1 // Size of storage buffer for ETI (is less as it is collected at very low freq)
#define STRBUFFERSIZE 128

////////////////////////////////////////////////
/// FSR analog input
///
#define FSR_NUMCHANNELS 1
#define PIN_FSR A1
#define FSR_FREQUENCY_HZ 100
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

///////////////////////////////////////////////////
/// SPI inputs
/// EMG SPI CONFIGURATION
#define EMG_NUMCHANNELS 2
#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 28

////////////////////////////////////////////////////
/// Serial inputs
/// Impedance and temperature using Serial port
#define ETI_NUMCHANNELS 1
#define ETI_SERIAL0 Serial4
#define ETI_SERIAL1 Serial3


// DEBUGGING FLAGS
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0
#define DEBUG_ADS1256

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "NFEris by ossip"


// Enable SD Card
#define SDCARD true

#endif
