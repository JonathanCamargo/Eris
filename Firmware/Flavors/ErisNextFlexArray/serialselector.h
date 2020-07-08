
#ifndef serialSELECTOR_H
#define serialSELECTOR_H

#include <Arduino.h>

#include "buffers.h"


namespace SerialSelector{

bool ReadSerial();

void start();
void SelectNegativeElectrode(uint8_t electrodeIdx);

}

#endif
