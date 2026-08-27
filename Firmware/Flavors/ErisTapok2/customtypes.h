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

// Sync uses bool here (vs uint8_t in eriscommon).
typedef struct boolSample{
  float timestamp;
  bool value;
} boolSample_t;

ERIS_DESCRIBE_BEGIN(boolSample_t)
  ERIS_TIME   (boolSample_t, timestamp)
  ERIS_META_U8(boolSample_t, value)
  ERIS_PAD    (boolSample_t, value)   // +1
  ERIS_PAD    (boolSample_t, value)   // +2
  ERIS_PAD    (boolSample_t, value)   // +3
ERIS_DESCRIBE_END(boolSample_t)
// This struct is NOT packed (unlike eriscommon's uint8_tSample_t), so the
// compiler pads it from 5 to 8 bytes and the binary protocol has always been
// sending those 3 bytes. They are declared here so the host decodes the same
// layout the firmware sends. Packing the struct instead would be a wire-format
// change -- it would need the host side changed in step.

#endif
