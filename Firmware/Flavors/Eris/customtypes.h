#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct FSRSample{
  float timestamp;
  float ch[FSR_NUMCHANNELS];
} FSRSample_t;

#endif
