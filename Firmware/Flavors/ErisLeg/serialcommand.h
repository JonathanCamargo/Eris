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

void TransmitFSR();
void TransmitJointState();
void TransmitLoadcellState();
void TransmitSineWave();
void TransmitSync();

void StreamSineWave();

void KillThreads();
void StartThreads();

void StreamingStart();
void StreamingStop();
void StreamingSetFeatures();
void stream();

void SetIP();
void SetIPA();
void SetIPK();


void GetError();

void GetIP();
void GetID();

void unrecognized();
void unrecognized(const char *command);




}

#endif
