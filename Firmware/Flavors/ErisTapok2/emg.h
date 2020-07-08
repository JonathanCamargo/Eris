#ifndef EMG_H
#define EMG_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace EMG{

extern ErisBuffer<EMGSample_t> buffer;
extern ErisBuffer<EMGSample_t> emgbuffer;

void start(void);
}

#endif
