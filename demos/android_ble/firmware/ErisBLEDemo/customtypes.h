#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <eris_descriptor.h>
#include <eris_customtypes.h>

// Three waveforms from one generator, so a single feature plots as a
// multi-series chart. The descriptor is what lets the phone app label the
// series without knowing this struct: it asks the firmware (DESC) and gets
//   DESC WAVES timestamp:f32:t,sine:f32,triangle:f32,square:f32
typedef struct {
  float timestamp;
  float sine;
  float triangle;
  float square;
} WavesSample_t;

ERIS_DESCRIBE_BEGIN(WavesSample_t)
  ERIS_TIME (WavesSample_t, timestamp)
  ERIS_FLOAT(WavesSample_t, sine)
  ERIS_FLOAT(WavesSample_t, triangle)
  ERIS_FLOAT(WavesSample_t, square)
ERIS_DESCRIBE_END(WavesSample_t)

// Two analog inputs, in volts. Described field by field rather than with
// eriscommon's AnalogSample<N>: that one's generated descriptor needs C++14,
// and the nRF52 core compiles as C++11.
typedef struct {
  float timestamp;
  float a0;
  float a1;
} AnalogPairSample_t;

ERIS_DESCRIBE_BEGIN(AnalogPairSample_t)
  ERIS_TIME (AnalogPairSample_t, timestamp)
  ERIS_FLOAT(AnalogPairSample_t, a0)
  ERIS_FLOAT(AnalogPairSample_t, a1)
ERIS_DESCRIBE_END(AnalogPairSample_t)

#endif
