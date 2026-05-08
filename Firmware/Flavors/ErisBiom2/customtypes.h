#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct BiomSample{
  float timestamp;
  float ch[BIOM_NUMCHANNELS];
} BiomSample_t;

#endif
