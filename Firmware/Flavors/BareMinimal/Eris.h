#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"

#ifdef ERIS_USE_FREERTOS
#include <Arduino.h>
#else
#include <ChRt.h>
#include <Arduino.h>
#endif

#include "customtypes.h"
#include <eriscommon.h>
#include <eris_streaming.h>

#include <eris_rtos.h>

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE];   // Buffer for str data for use across all files

//Mutex to take control of the LED
extern eris_mutex_t mtxhb;


#endif
