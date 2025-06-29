#ifndef IMU_H
#define IMU_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace IMU{

extern ErisBuffer<float> ** buffer;
//extern time_measurement_t t;


void start(void);
}

#endif
