#include "Eris.h"
#include "emgpill.h"

static const uint8_t emg_pins[EMG_NUMCHANNELS] = {PINS_EMG};

// Every pin must be on ADC1 (a no-op off ESP32). Add a line per channel if
// you raise EMG_NUMCHANNELS or change PINS_EMG.
ERIS_PIN_ASSERT_ADC1(ERIS_ADC(0));
ERIS_PIN_ASSERT_ADC1(ERIS_ADC(1));

#if !defined(ERIS_BOARD_ESP32)
static const float EMG_COUNTS_FS = (float)((1UL << EMG_ADC_BITS) - 1);
#endif

static inline float readVolts(uint8_t pin){
#if defined(ERIS_BOARD_ESP32)
  // eFuse-calibrated. Default attenuation (11 dB) covers ~0-3.1 V.
  return (float)analogReadMilliVolts(pin) * 0.001f;
#else
  return (float)analogRead(pin) * EMG_VREF / EMG_COUNTS_FS;
#endif
}

namespace EMGPill{

void begin(void){
#if !defined(ARDUINO_ARCH_AVR)
  analogReadResolution(EMG_ADC_BITS);
#endif
  for (uint8_t i = 0; i < EMG_NUMCHANNELS; i++) pinMode(emg_pins[i], INPUT);

  start();

  eriscommon::print("EMG Pill: ");
  eriscommon::print((double)EMG_NUMCHANNELS, 0);
  eriscommon::print(" ch @ ");
  eriscommon::print((double)EMG_HZ, 0);
  eriscommon::println(" Hz");
}

}

// One thread for all channels: they share the ADC, and a single owner keeps
// the per-tick reads back to back (tens of microseconds apart).
ERIS_SENSOR(EMGPill, EMGPillSample_t, EMG_HZ) {
  for (uint8_t i = 0; i < EMG_NUMCHANNELS; i++) s.ch[i] = readVolts(emg_pins[i]);
  return true;
}
