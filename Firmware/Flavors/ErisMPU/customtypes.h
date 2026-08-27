#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors

typedef struct IMUSample{
  float timestamp;
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
} IMUSample_t;

// Field descriptor. Keep it directly under the struct: if you add a field here
// and not there (or vice versa) ERIS_DESCRIBE_END fails the build rather than
// letting a column go silently missing.
ERIS_DESCRIBE_BEGIN(IMUSample_t)
  ERIS_TIME (IMUSample_t, timestamp)
  ERIS_FLOAT(IMUSample_t, ax)
  ERIS_FLOAT(IMUSample_t, ay)
  ERIS_FLOAT(IMUSample_t, az)
  ERIS_FLOAT(IMUSample_t, wx)
  ERIS_FLOAT(IMUSample_t, wy)
  ERIS_FLOAT(IMUSample_t, wz)
ERIS_DESCRIBE_END(IMUSample_t)

#endif
