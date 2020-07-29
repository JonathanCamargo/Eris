#ifndef FSR_H
#define FSR_H

#include <Arduino.h>

#include "Eris.h"
#include <eriscommon.h>

namespace FSR{

extern ErisBuffer<FSRSample_t> buffer;

void start(void);
}

#endif
