#ifndef SDCARD_H
#define SDCARD_H

#include "Eris.h"

namespace SDCard{

extern char * DEFAULT_TRIALNAME;

void addFSR(float value);
void addSine(float value);
void addEEG(float value,uint8_t chan);

void start(void);
void StopRecording(void);
void StartRecording(void);
void setTrialName(char * newtrialname);
}

#endif
