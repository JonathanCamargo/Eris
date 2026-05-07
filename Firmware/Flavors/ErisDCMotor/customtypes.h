#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>

typedef struct PotentiometerSample{
  float timestamp;
  float ch[POT_NUMCHANNELS];
} PotentiometerSample_t;

#endif
