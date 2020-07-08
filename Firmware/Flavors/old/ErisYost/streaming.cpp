#include "Eris.h"
#include "streaming.h"

#include "emg.h"
#include "features.h"
#include "sinewave.h"

#include <string.h>

namespace Streaming{
#define MAXSTRCMP 4
#define MAXFNC 10
static void (*streamfnc[MAXFNC])();
uint8_t Nfunctions=0;

static Packet packet;

void ClearFunctions(){
  Nfunctions=0;
}

bool AddFunction(char * arg){  
    Serial.print("Function requested: ");
    Serial.println(arg);
  if (!strncmp("SineWave",arg,MAXSTRCMP)){
      Serial.println("SineWave selected");
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("EMG",arg,MAXSTRCMP)){
      Serial.println("EMG selected");
    streamfnc[Nfunctions]=&EMG;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("FEAT",arg,MAXSTRCMP)){
      Serial.println("RMS features selected");
    streamfnc[Nfunctions]=&Feats;
    Nfunctions=Nfunctions+1;
  }
  else {
    return false;
  }
  if (Nfunctions>MAXFNC){
    Error::RaiseError(MEMORY,(char *)"STREAMFNC");
  }
  return true;
}

void SineWave(){
  //Transmit SineWave buffer
  float dest1[TXBUFFERSIZE];
  uint8_t num=SineWave::buffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
   packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float));
}

void EMG(){
  //Transmit SineWave buffer
  float dest1[TXBUFFERSIZE];
  for (uint8_t chan=0;chan<NUMEMGCHANNELS;chan++){  
    uint8_t num=EMG::buffer[chan]->FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
    for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
      dest1[i]=NAN;    
    }
    packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float)); 
  }
  }

void Feats(){
  //Transmit last feature vector computed
  packet.append((uint8_t *)Features::lastFeatures,FEATURESSIZE*sizeof(float)); 
} 



void Stream(){
  packet.start(Packet::PacketType::DATA);
  //Fetch data from desired buffers and send via serial
  for (uint8_t i=0;i<Nfunctions;i++){
    (*streamfnc[i])();
  }
   packet.send();
}

}
