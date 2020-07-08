#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

#define FIRMWARE_VERSION "v2.0" 

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED 
/// (classic arduino led)
/// 
#ifndef PIN_LED
#define PIN_LED 13 //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define FSR_TXBUFFERSIZE 10
#define TXBUFFERSIZE 16 // Size for transmision buffer last 16 points
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define STRBUFFERSIZE 128

////////////////////////////////////////////////
/// FSR analog input
///
#define NUMFSRCHANNELS 1
#define PIN_FSR A1
#define FSR_FREQUENCY_HZ 100
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

///////////////////////////////////////////////////
/// SPI inputs
/// EMG SPI CONFIGURATION
#define NUMEMGCHANNELS 8
#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 28

////////////////////////////////////////////////////
/// Negative electrode selector
#define EMG_SERIAL Serial3 // Serial3 are pins 7 RX3 and  8 TX3

// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


// Enable SD Card
#define SDCARD true

#endif
