#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"

namespace SDCard{

extern char * DEFAULT_TRIALNAME;

void addEMG(float emg, float currTime, uint8_t chan);
void addFSR(float value, float currTime, uint8_t chan);
void addSine(float);

void start(void);
void StopRecording(void);
bool StartRecording(void);
void setTrialName(char * newtrialname);
char* getTrialName();

extern long startTime; //Time when the record of trial starts
}

#endif
