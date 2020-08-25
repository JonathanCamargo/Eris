#include "Eris.h"
#include "streaming.h"

#include "emg.h"
#include "fsr.h"
#include "sinewave.h"
#include "sync.h"

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
    eriscommon::print("Function requested: ");
    eriscommon::println(arg);
  if (!strncmp("SINEWAVE",arg,MAXSTRCMP)){
      eriscommon::println("SineWave selected");
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("EMG",arg,MAXSTRCMP)){
      eriscommon::println("EMG selected");
    streamfnc[Nfunctions]=&EMG;
    Nfunctions=Nfunctions+1;
  }  
  else if (!strncmp("SYNC",arg,MAXSTRCMP)){
      eriscommon::println("Sync selected");
      streamfnc[Nfunctions]=&Sync;
      Nfunctions=Nfunctions+1;
  }  
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    eriscommon::println("FSR selected");
    streamfnc[Nfunctions]=&FSR;
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
  StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
}

void EMG(){
   StreamSamples<EMGSample_t,EMG_TXBUFFERSIZE>(EMG::buffer,packet);
}

void FSR(){
   StreamSamples<FSRSample_t,FSR_TXBUFFERSIZE>(FSR::buffer,packet);
}

void Sync(){
   // StreamSamples<uint8_tSample_t,SYNC_TXBUFFERSIZE>(Sync::buffer,packet);
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
