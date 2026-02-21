#ifndef SINEWAVE_H
#define SINEWAVE_H

#include <Arduino.h>

#include "Eris.h"
#include <eriscommon.h>

namespace SineWave{

extern long missed;
extern ErisBuffer<floatSample_t> buffer;
void start(void);
}

#endif
