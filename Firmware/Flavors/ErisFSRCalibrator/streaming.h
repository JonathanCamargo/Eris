
#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>
#include "Eris.h"
/*
All the functions related to streaming of data
*/


namespace Streaming{

bool AddFunction(char * arg);
void ClearFunctions();

//Functions should be declared as void and send the desired data via serial

// Sinusoidal wave
void SineWave();

// FSR voltage + load-cell force + axis position. Selected with "S_F FSR".
// Not named FSRCal: a function with that name would hide the FSRCal::
// namespace that owns the buffer ("expected primary-expression").
void CalStream();

void Stream();

}


#endif
