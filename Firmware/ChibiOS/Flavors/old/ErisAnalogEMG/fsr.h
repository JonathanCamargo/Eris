#ifndef FSR_H
#define FSR_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace FSR{

extern ErisBuffer<float> ** buffer;

void start(void);
}

#endif
