#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0"

/***********************************************/
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

/////////////////////////////////////////////////////
/// BLE TRANSPORT (nRF52 only)
/// Uncomment ERIS_USE_BLE to stream over Bluetooth LE (Nordic UART Service)
/// instead of USB serial. Works from the Arduino IDE -- the .ino reads these
/// defines and calls SerialCom::useBLE() (the eriscommon library is compiled
/// separately and never sees this header, so it can't be gated by a #define).
/// ERIS_BLE_NAME is the advertised device name the host scans for.
#define ERIS_USE_BLE
#ifndef ERIS_BLE_NAME
#define ERIS_BLE_NAME "Eris-Bare"
#endif

/////////////////////////////////////////////////////
//
// SYNC CONFIGURATION (Sync is a digital input that records time on change
//
#define PIN_SYNC 29
#define SYNC_TXBUFFERSIZE 24


// DEBUGGING FLAGS
#define DEBUG true
//#define DEBUG_TIME false
#define DEBUG_SYSCLK 180000000.0

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "Eris by ossip"


#endif
