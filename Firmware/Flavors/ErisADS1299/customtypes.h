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

#endif
