#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"

#include "customtypes.h"
#include <eriscommon.h>
#include <eris_streaming.h>
#include <eris_sd.h>

#include "features.h"




#include <Arduino.h>

#include <ChRt.h>

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE]; // Buffer for str data for use across all files
extern long t0; // Global time

//Mutex to take control of the LED
extern mutex_t mtxhb;


#endif
