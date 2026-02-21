#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>

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
/// FSR analog input
///
#define FSR_TXBUFFERSIZE 10 // Max samples to transmit in streaming
#define FSR_NUMCHANNELS 1
#define PIN_FSR A1
#define FSR_FREQUENCY_HZ 1000
#define FSR_PERIOD_US ((1.0/FSR_FREQUENCY_HZ)*1000000)

///////////////////////////////////////////////////
/// 
/// EMG CONFIGURATION
#define EMG_TXBUFFERSIZE 24 // Max samples to transmit in streaming
#define EMG_NUMCHANNELS 8
#define EMG_GAIN 2
#define CANTAPOK Can0

////////////////////////////////////////////////
///
/// ELMO DRIVER
#define ELMO_CANID0 126 // Knee Elmo
#define ELMO_CANID1 127 // Ankle Elmo

#define ELMO_MINCURRENT -15.0  // (A)
#define ELMO_MAXCURRENT 15.0   // (A)
#define ELMO_KTCONSTANT 0.038     // (Nm/A)
#define ELMO_RATE_US 4000      // Elmo update rate in microseconds
#define ELMO_MAX_COUNTS_INCREMENTAL 16384    // Maximum counts for the incremental encoder

////////////////////////////////////////////////
///
/// SRI LOADCELL DRIVER
#define SRILOADCELL_CANID 128  //Foot loadcell
#define SRILOADCELL_RATE_US 10000      // Elmo update rate in microseconds
////////////////////////////////////////////////

////////////////////////////////////////////////
///
/// MECHANICAL TRANSMISION
#define KNEE_GEAR_RATIO 120
#define ANKLE_GEAR_RATIO 175
/***********************************************/

///////////////////////////////////////////////////
/// 
/// JOINTS CONFIGURATION
#define JOINTS_TXBUFFERSIZE 10 // Max samples to transmit in streaming


///////////////////////////////////////////////////
/// 
/// LOADCELL CONFIGURATION
#define LOADCELL_TXBUFFERSIZE 10 // Max samples to transmit in streaming


////////////////////////////////////////////////////
///
///  IMU CONFIGURATION
#define IMU_TXBUFFERSIZE 10
#define PIN_IMU_TRUNK 8
#define PIN_IMU_THIGH 7
#define PIN_IMU_SHANK 6
#define PIN_IMU_FOOT  5
#define IMU_FREQUENCY_HZ 250.0
#define IMU_PERIOD_US ((1.0/IMU_FREQUENCY_HZ)*1000000)

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 29
#define SYNC_TXBUFFERSIZE 24


// DEBUGGING FLAGS
#define DEBUG true
#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"

#endif
