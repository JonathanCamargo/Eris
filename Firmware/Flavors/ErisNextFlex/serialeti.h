
#ifndef serialETI_H
#define serialETI_H

#include <Arduino.h>

#include "buffers.h"


namespace SerialETI{

bool ReadSerial(uint8_t idx);

void start(void);

extern ErisBuffer<TISample_t> buffer;

}

#endif
