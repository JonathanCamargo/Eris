#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors
#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>

typedef struct multiSample{
  float timestamp;
  float value[NUMCHANNELS];
} multiSample_t;

ERIS_DESCRIBE_CHANNELS(multiSample_t, value, NUMCHANNELS)

#endif
