#include "Eris.h"
#include "streaming.h"

#include "sinewave.h"

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
    eriscommon::print("Function requested: ");
    eriscommon::println(arg);
  #endif
  if (!strncmp("SINE",arg,MAXSTRCMP)){
    #if DEBUG
      eriscommon::println("SineWave selected");
    #endif
    streamfnc[Nfunctions]=&SineWave;
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

void SineWave(){
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
    StreamSamples<multiSample_t,TXBUFFERSIZE>(SineWave::buffer2,packet);
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
