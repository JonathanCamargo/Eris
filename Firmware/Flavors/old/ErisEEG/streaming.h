
#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>

/*
All the functions related to streaming of data
*/

namespace Streaming{
bool AddFunction(char * arg);
void ClearFunctions();
void Stream();
}

#endif
