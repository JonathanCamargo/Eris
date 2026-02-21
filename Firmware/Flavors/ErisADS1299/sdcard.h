#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"
#include <SD.h>

namespace SDCard{

extern char * DEFAULT_TRIALNAME;


extern ErisBuffer<FSRSample_t> fsrbuffer;
extern ErisBuffer<floatSample_t> sinebuffer;
extern ErisBuffer<uint8_tSample_t> syncbuffer;


void start(void);
void ResetTime(void);
void StopRecording(void);
bool StartRecording(void);
void setTrialName(char * newtrialname);
char* getTrialName();

extern long startTime; //Time when the record of trial starts

}

#endif
