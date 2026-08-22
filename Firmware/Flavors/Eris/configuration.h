#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / ERIS_SERIAL_BAUD

#define FIRMWARE_VERSION "v3.0" 

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED
/// PIN_LED comes from eris_board.h (13 on Teensy, 2 on ESP32). Define it
/// before that include to override.
///
#if defined(ERIS_BOARD_ESP32)
#define PIN_LED_R 4      // any free output GPIO
#else
#define PIN_LED_R 28
#endif
ERIS_PIN_ASSERT_OUTPUT(PIN_LED_R);

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define SDBUFFERSIZE_FSR 100
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 2
// ERIS_ADC(n), not A0/A1: the ESP32 variant has no A1 at all, and its A10..A15
// are ADC2 pins that stop converting once WiFi is up. See eris_board.h.
#define PIN_FSR_0 ERIS_ADC(0)
#define PIN_FSR_1 ERIS_ADC(1)
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

///////////////////////////////////////////////////
/// 
/// EMG CONFIGURATION
#define EMG_TXBUFFERSIZE 24 // Max samples to transmit in streaming
#define EMG_NUMCHANNELS 8
#define EMG_GAIN 2
#define CANTAPOK Can0

////////////////////////////////////////////////////
///
///  IMU CONFIGURATION
#define IMU_TXBUFFERSIZE 10
#if defined(ERIS_BOARD_ESP32)
// Teensy's 8/7/6/5 are unusable here: GPIO 6..11 are the SPI flash bus.
#define PIN_IMU_TRUNK 17
#define PIN_IMU_THIGH 16
#define PIN_IMU_SHANK 5
#define PIN_IMU_FOOT  18
#else
#define PIN_IMU_TRUNK 8
#define PIN_IMU_THIGH 7
#define PIN_IMU_SHANK 6
#define PIN_IMU_FOOT  5
#endif
#define IMU_FREQUENCY_HZ 250.0
#define IMU_PERIOD_US ((1.0/IMU_FREQUENCY_HZ)*1000000)

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#if defined(ERIS_BOARD_ESP32)
#define PIN_SYNC 27      // input with pull-up; ADC2 is fine for a digital pin
#else
#define PIN_SYNC 29
#endif
#define SYNC_TXBUFFERSIZE 24


// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#if defined(ERIS_BOARD_ESP32)
#define DEBUG_SYSCLK 240000000.0
#else
#define DEBUG_SYSCLK 180000000.0
#endif

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


// Enable SD Card
#define SDCARD true

#endif
