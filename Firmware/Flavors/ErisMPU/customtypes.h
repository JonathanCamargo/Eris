#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>

typedef struct IMUSample{
  float timestamp;
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
} IMUSample_t;

#endif
