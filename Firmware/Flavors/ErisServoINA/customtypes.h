#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

// INA219 current sensor sample (bus voltage, shunt voltage, current, power)
typedef struct __attribute((__packed__)) inaSample{
  float timestamp;
  float current_mA;
  float busVoltage_V;
  float power_mW;
} inaSample_t;

#endif
