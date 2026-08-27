#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors
#include <eris_customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>

// INA219 current sensor sample (bus voltage, shunt voltage, current, power)
typedef struct __attribute((__packed__)) inaSample{
  float timestamp;
  float current_mA;
  float busVoltage_V;
  float power_mW;
} inaSample_t;

ERIS_DESCRIBE_BEGIN(inaSample_t)
  ERIS_TIME (inaSample_t, timestamp)
  ERIS_FLOAT(inaSample_t, current_mA)
  ERIS_FLOAT(inaSample_t, busVoltage_V)
  ERIS_FLOAT(inaSample_t, power_mW)
ERIS_DESCRIBE_END(inaSample_t)

#endif
