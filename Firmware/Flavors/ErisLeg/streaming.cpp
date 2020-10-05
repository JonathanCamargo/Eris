#include "Eris.h"
#include "streaming.h"
#include "configuration.h"
#include "fsr.h"
#include "sinewave.h"
#include "sync.h"
#include "joints.h"
#include "loadcell.h"

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
  if (!strncmp("KNEE",arg,MAXSTRCMP)){
    #if DEBUG
      eriscommon::println("Knee selected");
    #endif
    streamfnc[Nfunctions]=&KneeJoint;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("ANKLE",arg,MAXSTRCMP)){
    #if DEBUG
      eriscommon::println("Ankle selected");
    #endif
    streamfnc[Nfunctions]=&AnkleJoint;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("LC",arg,MAXSTRCMP)){
    #if DEBUG
      eriscommon::println("Loadcell selected");
    #endif
    streamfnc[Nfunctions]=&Loadcell;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("SINE",arg,MAXSTRCMP)){
    #if DEBUG
      eriscommon::println("SineWave selected");
    #endif
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    eriscommon::println("FSR selected");
    streamfnc[Nfunctions]=&FSR;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("SYNC",arg,MAXSTRCMP)){
    eriscommon::println("SYNC selected");
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

void SineWave(){
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
}

void FSR(){
    StreamSamples<FSRSample_t,FSR_TXBUFFERSIZE>(FSR::buffer,packet);
}


void KneeJoint(){
    StreamSamples<JointStateSample_t,JOINTS_TXBUFFERSIZE>(Joints::kneebuffer,packet);
}

void AnkleJoint(){
    StreamSamples<JointStateSample_t,JOINTS_TXBUFFERSIZE>(Joints::anklebuffer,packet);
}

void Loadcell(){
    StreamSamples<LoadcellSample_t,LOADCELL_TXBUFFERSIZE>(Loadcell::buffer,packet);
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
