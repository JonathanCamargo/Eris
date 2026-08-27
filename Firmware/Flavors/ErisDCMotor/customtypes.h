#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors
#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>

typedef struct PotentiometerSample{
  float timestamp;
  float ch[POT_NUMCHANNELS];
} PotentiometerSample_t;

ERIS_DESCRIBE_CHANNELS(PotentiometerSample_t, ch, POT_NUMCHANNELS)

#endif
