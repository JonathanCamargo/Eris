#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct BiomSample{
  float timestamp;
  float ch[BIOM_NUMCHANNELS];
} BiomSample_t;

// Sync uses bool (vs uint8_t in other flavors). Kept separate from eriscommon
// to preserve wire format for this flavor's existing host-side parsers.
typedef struct boolSample{
  float timestamp;
  bool value;
} boolSample_t;

#endif
