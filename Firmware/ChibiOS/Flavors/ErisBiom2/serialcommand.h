
#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <Arduino.h>


#include <SerialCommand.h>  // Due to the way the Arduino IDE compiles

namespace SerialCom{

void start(void);


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

void getClassificationFeatures();
void updateRegVals(); 
void enableFeatures();
void disableFeatures();

extern bool stream_en;
extern bool classifQuery;

}

#endif
