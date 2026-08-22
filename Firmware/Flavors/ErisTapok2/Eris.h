#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"
#include <eris_flavor.h>   // RTOS detect + ChRt + Arduino + eriscommon + streaming

#include "customtypes.h"

#if SDCARD
#include <eris_sd.h>
#endif

extern const char firmwareInfo[];

#endif
