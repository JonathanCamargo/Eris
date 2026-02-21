#ifndef EMG_H
#define EMG_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace EMG{

extern ErisBuffer<float> **tBuffer; 
extern ErisBuffer<float> ** buffer;
void start(void);

}

#endif
