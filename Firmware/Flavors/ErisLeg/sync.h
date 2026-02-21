#ifndef SYNC_H
#define SYNC_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace Sync{

extern ErisBuffer<uint8_tSample_t> buffer; 

void start(void);
}

#endif
