
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

// Axis state: position, velocity, target, driver status. Selected with
// "S_F STEP". Named StepperAxis rather than Stepper on purpose: a function
// named Stepper here would hide the Stepper:: namespace that owns the buffer,
// and the error you get for that ("expected primary-expression") says nothing
// about the real cause.
void StepperAxis();

void Stream();

}


#endif
