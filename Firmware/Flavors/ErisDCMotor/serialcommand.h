#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <Arduino.h>


#include <SerialCommand.h>  // Due to the way the Arduino IDE compiles

namespace SerialCom{

void start(void);

extern long startTime;

// Callback function for specific //Serial commands
void INFO();
void LED_on();
void LED_off();
void SayHello();

void TransmitSineWave();
void TransmitPotentiometer();
void StreamSineWave();
void StreamingSetFeatures();
void SynchronizeTime();

void KillThreads();
void StartThreads();

void StreamingStart();
void StreamingStop();
void stream();


void GetError();

void unrecognized();
void unrecognized(const char *command);




}

#endif
