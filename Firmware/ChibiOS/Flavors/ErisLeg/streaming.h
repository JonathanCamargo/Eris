
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



void KneeJoint();
void AnkleJoint();

void FSR();
void Loadcell();
void SineWave(); // Sinusoidal wave

void Sync();

void Stream();

}


#endif
