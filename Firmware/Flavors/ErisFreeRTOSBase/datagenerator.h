#pragma once

#include <Arduino.h>

#include "Eris.h"
#include <buffers.h>

namespace DataGenerator{

extern long missed;
extern ErisBuffer<floatSample_t> buffer;
void start(void);
void Task_DataGenerator(void *pvParameters);
void PrintSample(floatSample_t sample);
}
