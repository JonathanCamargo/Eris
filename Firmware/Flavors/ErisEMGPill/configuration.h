#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>  // board detection + PIN_LED / ERIS_ADC / STRINGIFY

#define FIRMWARE_VERSION "v3.0"
#define MAXSTRCMP 8  // Maximum length for string comparison

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// LED
/// (classic arduino led; on the XIAO ESP32S3 eris_board.h picks GPIO 21)
///
#ifndef PIN_LED
#define PIN_LED 13
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16  // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// EMG Pill analog front-ends
///
/// Each pill is a single-channel analog EMG amplifier whose OUT pin sits at
/// mid-supply (VCC/2) at rest and swings around it with muscle activity.
///
///     3V3 ---- VCC  [ EMG Pill ]  OUT ---- ERIS_ADC(n)
///     GND ---- GND               (REF/IN+/IN- to the electrodes)
///
/// Power the pills from 3V3, never 5 V: OUT can swing to the rail, and the
/// ADC pin cannot take more than its own supply.
///
/// ERIS_ADC(n) on the XIAO ESP32S3 is GPIO n+1, i.e. silk A0/D0, A1/D1, ...
/// (all on ADC1, so analogRead keeps working with WiFi on). Do not use the
/// A0/A1 aliases directly -- see doc/ADDING_A_SENSOR.md.
#define EMG_NUMCHANNELS 2
#define PINS_EMG ERIS_ADC(0), ERIS_ADC(1)

/// Sampling rate in Hz. Must divide 1000 (ERIS_SENSOR static_asserts it).
/// Surface EMG has most of its energy at 20-450 Hz; 500 Hz captures the
/// envelope and most of the band. 1000 Hz is the ERIS_SENSOR ceiling.
#define EMG_HZ 500

/// ADC resolution, and full-scale for the non-ESP32 path (counts -> volts).
/// On ESP32 the firmware uses analogReadMilliVolts(), which applies the
/// factory eFuse calibration and is far more linear than counts * 3.3/4095.
#define EMG_VREF 3.3f
#define EMG_ADC_BITS 12

/// Max samples per streaming packet. Streaming runs every 10 ms, so at 500 Hz
/// ~5 samples are waiting each time; 16 leaves headroom for a slow host.
#define EMG_TXBUFFERSIZE 16

// DEBUGGING FLAGS
#define DEBUG true

#define FIRMWARE_INFO ERIS_FIRMWARE_INFO("ErisEMGPill")

#endif
