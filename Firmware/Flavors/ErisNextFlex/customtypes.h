#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors
#include <eris_customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct EMGSample{
  float timestamp;
  float ch[EMG_NUMCHANNELS];
} EMGSample_t;

ERIS_DESCRIBE_CHANNELS(EMGSample_t, ch, EMG_NUMCHANNELS)

typedef struct FSRSample{
  float timestamp;
  float ch[FSR_NUMCHANNELS];
} FSRSample_t;

ERIS_DESCRIBE_CHANNELS(FSRSample_t, ch, FSR_NUMCHANNELS)

typedef struct TISample{
  float timestamp;
  float temperature[ETI_NUMCHANNELS];
  float impedance[ETI_NUMCHANNELS];
} TISample_t;

ERIS_DESCRIBE_BEGIN(TISample_t)
  ERIS_TIME (TISample_t, timestamp)
  ERIS_FLOAT(TISample_t, temperature)
  ERIS_FLOAT(TISample_t, impedance)
ERIS_DESCRIBE_END(TISample_t)
// Valid only while ETI_NUMCHANNELS == 1: offsetof(member) is then the single
// element. Raise the count and ERIS_DESCRIBE_END fails the build, which is the
// signal to generate this table instead of listing it.

#endif
