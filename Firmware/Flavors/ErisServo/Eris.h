#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"

// RTOS include: tells Arduino build system to link ChRt (ChibiOS for Teensy)
// eris_rtos.h provides the portable API on top of this
#include <ChRt.h>

#include "customtypes.h"
#include <eriscommon.h>
#include <eris_streaming.h>

#include <Arduino.h>

#include <eris_rtos.h>

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE];   // Buffer for str data for use across all files

extern long t0; // Global start time for all modules

//Mutex to take control of the LED
extern eris_mutex_t mtxhb;


#endif
