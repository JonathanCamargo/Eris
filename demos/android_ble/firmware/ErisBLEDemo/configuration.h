#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0"
#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) " Eris BLE demo"

/////////////////////////////////////////////////////
/// BLE TRANSPORT (nRF52 only)
/// The Android demo app scans for the Nordic UART Service, so any name works;
/// this is just what shows up in the device list.
#define ERIS_USE_BLE
#ifndef ERIS_BLE_NAME
#define ERIS_BLE_NAME "Eris-Demo"
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16   // max samples per feature per streamed packet
#define STRBUFFERSIZE 64
#define MAXSTRCMP 8       // feature names are compared up to this many chars

/////////// DEMO SIGNALS
#define WAVES_RATE_HZ  100   // sine / triangle / square generator
#define ANALOG_RATE_HZ 50    // A0 + A1
#define TEMP_RATE_HZ   10    // nRF52 die temperature

#define PIN_ANALOG_0 ERIS_ADC(0)
#define PIN_ANALOG_1 ERIS_ADC(1)

/////////// RGB LED (Seeed XIAO nRF52840)
// The XIAO's RGB LED is common-anode: driving a pin LOW lights it. If your
// board lights on HIGH, swap these two.
#define RGB_LED_ON  LOW
#define RGB_LED_OFF HIGH

#endif
