#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct EMGSample{
  float timestamp;
  float ch[EMG_NUMCHANNELS];
} EMGSample_t;

typedef struct FSRSample{
  float timestamp;
  float ch[FSR_NUMCHANNELS];
} FSRSample_t;

#endif
