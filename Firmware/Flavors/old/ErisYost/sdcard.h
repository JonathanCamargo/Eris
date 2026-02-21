#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"

namespace SDCard{

extern long startTime;

void addSine(float);
void addEMG(float emg, float currTime, uint8_t chan);
void start(void);
void StopRecording(void);
void StartRecording(void);

extern char DEFAULT_FILENAME[10];
extern char filename[10];

//extern time_measurement_t t;

}

#endif
