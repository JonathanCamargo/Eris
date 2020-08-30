#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <Arduino.h>

#include "Eris.h"
#include <eriscommon.h>

namespace Potentiometer{

extern long missed;
extern ErisBuffer<PotentiometerSample_t> buffer;

void start(void);
}

#endif
