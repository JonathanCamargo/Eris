#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

typedef struct FSRSample{
  float timestamp;
  float ch[FSR_NUMCHANNELS];
} FSRSample_t;

typedef struct ImpedanceParameters{
  float timestamp;
  float k;
  float b;
  float theta_eq;
} ImpedanceParameters_t;

typedef struct JointStateSample{
  float timestamp;
  float theta;
  float theta_dot;
  float torque;
} JointStateSample_t;

typedef struct LoadcellSample{
  float timestamp;
  float forceX;
  float forceY;
  float forceZ;
  float momentX;
  float momentY;
  float momentZ;
} LoadcellSample_t;

#endif
