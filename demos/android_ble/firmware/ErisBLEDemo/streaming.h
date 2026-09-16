#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>
#include "Eris.h"

namespace Streaming {

bool AddFunction(char * arg);
void ClearFunctions();
void Stream();

}

#endif
