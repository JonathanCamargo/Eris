#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

#define FIRMWARE_VERSION "v2.0" 
#define TEENSY3 //TEENSY4

#define MAXSTRCMP 5 // Maximum length for string comparison

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
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 1
#define PIN_FSR A1
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)


////////////////////////////////////////////////////
///
///  IMU CONFIGURATION
#define IMU_TXBUFFERSIZE 10
#ifdef TEENSY4
#define PIN_IMU_TRUNK 2   // Teensy 4.0
#define PIN_IMU_THIGH 3   // 3
#define PIN_IMU_SHANK 4
#define PIN_IMU_FOOT  5
#endif
#ifdef TEENSY3
#define PIN_IMU_TRUNK 8   // Teensy 3.6
#define PIN_IMU_THIGH 7   // 3
#define PIN_IMU_SHANK 6
#define PIN_IMU_FOOT  5
#endif
#define IMU_FREQUENCY_HZ 250.0
#define IMU_PERIOD_US ((1.0/IMU_FREQUENCY_HZ)*1000000)

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 28

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

//////////////////// FEATURES 
// Enable Features 
#define FEATURES true

// Timing
#define FEATURES_REGRESSION_WINDOW_MS 250
#define FEATURES_REGRESSION_PERIOD_MS 50
#define FEATURES_CLASSIFICATION_WINDOW_MS 100


#endif
