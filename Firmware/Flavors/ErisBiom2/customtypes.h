#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors
#include <eris_customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct BiomSample{
  float timestamp;
  float ch[BIOM_NUMCHANNELS];
} BiomSample_t;

ERIS_DESCRIBE_CHANNELS(BiomSample_t, ch, BIOM_NUMCHANNELS)

#endif
