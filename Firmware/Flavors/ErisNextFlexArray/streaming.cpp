#include "Eris.h"
#include "streaming.h"

#include "emg.h"
#include "fsr.h"
#include <modules/sinewave.h>

#include <string.h>

namespace Streaming{
#define MAXSTRCMP 4
#define MAXFNC 10
static void (*streamfnc[MAXFNC])();
uint8_t Nfunctions=0;

void ClearFunctions(){
  Nfunctions=0;
}

bool AddFunction(char * arg){
  #if DEBUG
    Serial.print("Function requested: ");
    Serial.println(arg);
  #endif
  if (!strncmp("SineWave",arg,MAXSTRCMP)){
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
  float dest1[TXBUFFERSIZE];
  for (uint8_t chan=0;chan<NUMEMGCHANNELS+1;chan++){
    if (chan ==0) {
        uint8_t num=EMG::buffer[0]->FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
    }
    else {
      uint8_t num=EMG::buffer[chan]->FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
      for (int i=num;i<TXBUFFERSIZE;i++){
        dest1[i]=NAN;
      }
    }
    packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float));
  }
}


void FSR(){
  float dest1[FSR_TXBUFFERSIZE];
  uint8_t q = 15;
  for (uint8_t chan=0;chan<NUMFSRCHANNELS+1;chan++){
    if (chan ==0) {
      uint8_t num=FSR::buffer[chan]->FetchData(dest1,(char *)"FSR",FSR_TXBUFFERSIZE);
    }
    else {
    uint8_t num=FSR::buffer[chan]->FetchData(dest1,(char *)"FSR",FSR_TXBUFFERSIZE);
      for (int i=num;i<FSR_TXBUFFERSIZE;i++){
        dest1[i]=NAN;
      }
    }
    packet.append((uint8_t *)dest1,FSR_TXBUFFERSIZE*sizeof(float));
  }
  packet.append(&q, 1);
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
