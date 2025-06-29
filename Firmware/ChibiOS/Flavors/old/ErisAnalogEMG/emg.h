#ifndef EMG_H
#define EMG_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace EMG{

extern ErisBuffer<float> ** buffer;
extern ErisBuffer<float> ** rmsbuffer;

void start(void);
}

#endif
