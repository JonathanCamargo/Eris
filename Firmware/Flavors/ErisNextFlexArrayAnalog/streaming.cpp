#include "Eris.h"
#include "streaming.h"

#include "emg.h"
#include "fsr.h"
#include <modules/sinewave.h>

#include <string.h>

namespace Streaming{
#define MAXSTRCMP 5
#define MAXFNC 10
static void (*streamfnc[MAXFNC])();
uint8_t Nfunctions=0;

void ClearFunctions(){
  Nfunctions=0;
}

#define DEBUG 1
bool AddFunction(char * arg){
  #ifdef DEBUG
    Serial.print("Function requested: ");
    Serial.println(arg);
  #endif
  if (!strncmp("SINE",arg,MAXSTRCMP)){
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("EMG",arg,MAXSTRCMP)){
    streamfnc[Nfunctions]=&EMG;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    streamfnc[Nfunctions]=&FSR;
    Nfunctions=Nfunctions+1;
  }
  else {
    return false;
  }
  if (Nfunctions>MAXFNC){
    Error::RaiseError(Error::MEMORY,(char *)"STREAMFNC");
  }
  return true;
}

void EMG(){
    StreamSamples<EMGSample_t,EMG_TXBUFFERSIZE>(EMG::buffer,packet);
}

void FSR(){
    StreamSamples<FSRSample_t,FSR_TXBUFFERSIZE>(FSR::buffer,packet);
}

void SineWave(){
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
}


void Stream(){
  packet.start(Packet::PacketType::DATA);
  for (uint8_t i=0;i<Nfunctions;i++){
    (*streamfnc[i])();
  }
  packet.send();
}

}
