#ifndef LOADCELL_H
#define LOADCELL_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>
#include "customtypes.h"
#include <sriloadcell.h>

namespace Loadcell{

extern ErisBuffer<LoadcellSample_t> buffer;

void start(void);
void kill(void);

}

#endif
