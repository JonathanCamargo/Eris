#ifndef EMG_H
#define EMG_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace Analog{

extern ErisBuffer<AnalogSample_t> buffer;
void start(void);

}

#endif
