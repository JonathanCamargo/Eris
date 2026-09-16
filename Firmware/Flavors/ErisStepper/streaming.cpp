#include "Eris.h"
#include "streaming.h"

#include "motion.h"
#include <modules/sinewave.h>

#include <string.h>

namespace Streaming{

#define MAXFNC 10

static void (*streamfnc[MAXFNC])();
static uint8_t Nfunctions=0;

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
  else if (!strncmp("STEP",arg,MAXSTRCMP)){
    #if DEBUG
      eriscommon::println("Stepper selected");
    #endif
    streamfnc[Nfunctions]=&StepperAxis;
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
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet,"SINE");
}

void StepperAxis(){
    StreamSamples<StepperSample_t,STEPPER_TXBUFFERSIZE>(Stepper::buffer,packet,"STEP");
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
