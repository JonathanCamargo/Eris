#ifndef PRESSURE_H
#define PRESSURE_H

#include <Arduino.h>
#include "buffers.h"

namespace Pressure{

extern long missed;
extern ErisBuffer<float> buffer;

#if FEATURES
extern ErisBuffer<float> rmsbuffer;
#endif
void start(void);
}

#endif
