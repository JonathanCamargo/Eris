
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
void test(); // DELETE THIS
void INFO();
void enableTPackets(); 
void LED_on();
void LED_off();
void SayHello();

void TransmitSineWave();
void TransmitFEAT();
void TransmitBiom();
void TransmitGait();
void StreamSineWave();
void TransmitSync();

void KillThreads();
void StartThreads();

void StreamingStart();
void StreamingStop();
void StreamingSetFeatures();
void stream();

void GetError();

void SynchronizeTime();

void StartRecording();
void StopRecording();

void unrecognized();
void unrecognized(const char *command);

void EnableFeatures();
void DisableFeatures();
}

#endif
