
#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>
#include "packet.h"

/*
All the functions related to streaming of data
*/


namespace Streaming{




bool AddFunction(char * arg);
void ClearFunctions();
//Functions should be declared as void and send the desired data via serial

void SineWave();
void Pressure();

#if FEATURES
void SineWaveRMS();
#endif

void Stream();

extern Packet packet;

}

#endif
