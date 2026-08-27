
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
// Must be declared here: AddFunction() takes &EMG above its definition, and
// without this declaration the name resolves to the EMG:: namespace instead,
// giving "expected primary-expression before ';'" pointing at the wrong line.
void EMG();
void FSR(); 

void Stream();

}


#endif
