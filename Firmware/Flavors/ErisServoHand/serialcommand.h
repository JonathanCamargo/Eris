#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <Arduino.h>
#include <SerialCommand.h>

namespace SerialCom{

void start(void);
void GetError();

void KillThreads();
void StartThreads();

// Callback function for specific //Serial commands
void INFO();
void LED_on();
void LED_off();
void SayHello();
void XParser();

///////////////////////////////////
// Console Data showing functions
//
void ShowDataGeneratorSamples();
void unrecognized();
void unrecognized(const char *command);
}

#endif
