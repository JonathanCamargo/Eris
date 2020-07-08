#ifndef ERIS_H
#define ERIS_H
#include "configuration.h"

#include "customtypes.h"
#include <eriscommon.h>
#include <eris_streaming.h>

#include <Arduino.h>

#include <ChRt.h>

void ResetTime();

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE]; // Buffer for str data for use across all files

extern mutex_t mtxhb;
extern long t0;

#endif
