#include "signals.h"
#include <eris_sensor.h>

namespace Demo {
  volatile float freqHz = 1.0f;
  volatile float amp    = 1.0f;
}

// Integrate phase rather than computing sin(2*pi*f*t), so changing FREQ from
// the phone bends the waveform smoothly instead of jumping.
static float wavesPhase = 0.0f;   // in cycles, [0, 1)

ERIS_SENSOR(Waves, WavesSample_t, WAVES_RATE_HZ) {
  wavesPhase += Demo::freqHz / (float)WAVES_RATE_HZ;
  wavesPhase -= (float)(int)wavesPhase;

  const float a = Demo::amp;
  s.sine     = a * sin(2.0f * M_PI * wavesPhase);
  s.triangle = a * (4.0f * fabs(wavesPhase - 0.5f) - 1.0f);
  s.square   = (wavesPhase < 0.5f) ? a : -a;
  return true;
}

ERIS_SENSOR(Analog, AnalogPairSample_t, ANALOG_RATE_HZ) {
  s.a0 = analogRead(PIN_ANALOG_0) * (3.3f / 4095.0f);
  s.a1 = analogRead(PIN_ANALOG_1) * (3.3f / 4095.0f);
  return true;
}

ERIS_SENSOR(Temp, floatSample_t, TEMP_RATE_HZ) {
#if defined(ERIS_BOARD_NRF52)
  s.value = readCPUTemperature();   // degrees C
  return true;
#else
  return false;                     // no on-chip sensor on this board
#endif
}
