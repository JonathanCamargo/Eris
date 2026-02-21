
#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>

/*
All the functions related to streaming of data
*/


namespace Streaming{

bool AddFunction(char * arg);
void ClearFunctions();

//Functions should be declared as void and send the desired data via serial

// Sinusoidal wave
void SineWave();
void EMG();
void FSR();


void Stream();


}

#endif
