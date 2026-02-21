#include "Eris.h"
#include "streaming.h"

#include "fsr.h"
#include "sinewave.h"
#include "sync.h"

#include <string.h>

namespace Streaming{
  
#define MAXSTRCMP 5
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
  if (!strncmp("SINE",arg,MAXSTRCMP)){
    #if DEBUG
      Serial.println("SineWave selected");
    #endif
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }  
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    Serial.println("FSR selected");
    streamfnc[Nfunctions]=&FSR;
    Nfunctions=Nfunctions+1;
  }  
  else if (!strncmp("SYNC",arg,MAXSTRCMP)){
    Serial.println("SYNC selected");
    streamfnc[Nfunctions]=&Sync;
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
void Hola()
{
   Serial.print("hola");
}

void SineWave(){
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
}

void FSR(){
    StreamSamples<FSRSample_t,FSR_TXBUFFERSIZE>(FSR::buffer,packet);
}
void Sync(){
    StreamSamples<uint8_tSample_t,SYNC_TXBUFFERSIZE>(Sync::buffer,packet);
}


void Stream(){
  //Fetch data from desired buffers and send via serial
  packet.start(Packet::PacketType::DATA); 
  for (uint8_t i=0;i<Nfunctions;i++){
    (*streamfnc[i])();
  }   
  packet.send(); 
}

}
