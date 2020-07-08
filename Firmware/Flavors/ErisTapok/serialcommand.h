
#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <Arduino.h>


#include <SerialCommand.h>  // Due to the way the Arduino IDE compiles

namespace SerialCom{

void start(void);


// Callback function for specific //Serial commands
void Info();
void LED_on();
void LED_off();
void ResetT0();
void SayHello();
void TransmitSineWave();
void TransmitEMG();
void TransmitFSR();
void TransmitSync();

void StreamSineWave();

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
