#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"

#include "customtypes.h"
#include <eriscommon.h>
#include <eris_streaming.h>
#include <eris_sd.h>

#include <Arduino.h>

#include <ChRt.h>

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE]; // Buffer for str data for use across all files
extern long t0; // Global start time for all modules

#endif
