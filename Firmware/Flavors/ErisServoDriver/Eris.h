#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"

// RTOS include: on ChibiOS targets this tells the Arduino build system to link
// ChRt. On nRF52 the core ships FreeRTOS and ChRt would clash (SVC_Handler),
// so it is skipped. eris_rtos.h provides the portable API on top of either.
#ifdef ERIS_USE_FREERTOS
#include <Arduino.h>
#else
#include <ChRt.h>
#endif

#include "customtypes.h"
#include <eriscommon.h>
#include <eris_streaming.h>

#include <Arduino.h>

#include <eris_rtos.h>

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE];   // Buffer for str data for use across all files

//Mutex to take control of the LED
extern eris_mutex_t mtxhb;


#endif
