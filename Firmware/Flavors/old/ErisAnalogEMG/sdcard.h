#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"

namespace SDCard{

extern char * DEFAULT_TRIALNAME;
extern char * trialname;
extern long startTime;

void addSine(float);
void addEMG(float emg, float currTime, uint8_t chan);
void addFSR(float fsr, float currTime, uint8_t chan);
void start(void);
void StopRecording(void);
void StartRecording(void);
void setTrialName(char * newtrialname);




}

#endif
