#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"
#include <SD.h>

namespace SDCard{

extern char * DEFAULT_TRIALNAME;


extern ErisBuffer<EMGSample_t> emgbuffer;

void addEMG(EMGSample_t sample);
void addFSR(FSRSample_t sample);
void addSine(floatSample_t sample);
void addSync(boolSample_t sample);

void start(void);
void StopRecording(void);
bool StartRecording(void);
void setTrialName(char * newtrialname);
char* getTrialName();

extern long startTime; //Time when the record of trial starts

}

#endif
