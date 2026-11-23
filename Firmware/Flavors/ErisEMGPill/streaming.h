
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

// EMG Pill channels. Selected with "S_F EMG".
// Not named EMGPill: a function with that name would hide the EMGPill::
// namespace that owns the buffer ("expected primary-expression").
void EMGStream();

void Stream();

}


#endif
