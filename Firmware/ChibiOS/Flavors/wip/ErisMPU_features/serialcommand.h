#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <Arduino.h>


#include <SerialCommand.h>  // Due to the way the Arduino IDE compiles

#include "serialcommand_features.h"


namespace SerialCom{

extern SerialCommand sCmd;

void start(void);

extern long startTime;

// Callback function for specific //Serial commands
void INFO();
void LED_on();
void LED_off();
void SayHello();

void ShowFailures();
void TransmitFSR();
void TransmitIMU();
void TransmitSineWave();
void TransmitSync();

void StreamSineWave();

void SynchronizeTime();

void KillThreads();
void StartThreads();

void StreamingStart();
void StreamingStop();
void StreamingSetFeatures();
void stream();


void GetError();

void StartRecording();
void StopRecording();

void unrecognized();
void unrecognized(const char *command);




}

#endif
