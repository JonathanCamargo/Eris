#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include "Eris.h"
#include <eriscommon.h>

namespace Motor{

void Forward();
void Backward();
void Idle();
void Break();
void UpdateMeasurement(double angle);
void start(void);
}

#endif
