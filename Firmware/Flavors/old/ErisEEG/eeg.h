#ifndef EEG_H
#define EEG_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace EEG{

extern ErisBuffer<float> ** buffer;

void start(void);
}

#endif
