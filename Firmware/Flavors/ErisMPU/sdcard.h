#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"
#include <SD.h>

namespace SDCard{

extern char * DEFAULT_TRIALNAME;


extern ErisBuffer<FSRSample_t> fsrbuffer;
extern ErisBuffer<IMUSample_t> imutrunkbuffer;
extern ErisBuffer<IMUSample_t> imuthighbuffer;
extern ErisBuffer<IMUSample_t> imushankbuffer;
extern ErisBuffer<IMUSample_t> imufootbuffer;
extern ErisBuffer<floatSample_t> sinebuffer;
extern ErisBuffer<boolSample_t> syncbuffer;


void start(void);
void ResetTime(void);
void StopRecording(void);
bool StartRecording(void);
void setTrialName(char * newtrialname);
char* getTrialName();

extern long startTime; //Time when the record of trial starts

}

#endif
