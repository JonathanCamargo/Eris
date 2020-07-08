#ifndef FSR_H
#define FSR_H

#include <Arduino.h>

#include "Eris.h"
#include <eriscommon.h>

namespace FSR{

extern long missed;
extern ErisBuffer<float> buffer;

void start(void);
}

#endif
