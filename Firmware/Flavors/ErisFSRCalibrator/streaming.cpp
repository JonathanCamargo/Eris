#include "Eris.h"
#include "streaming.h"

#include "loadcell.h"
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
  if (Nfunctions>=MAXFNC){
    Error::RaiseError(Error::MEMORY,(char *)"STREAMFNC");
    return false;
  }
  if (!strncmp("SINE",arg,MAXSTRCMP)){
    streamfnc[Nfunctions++]=&SineWave;
  }
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    streamfnc[Nfunctions++]=&CalStream;
  }
  else {
    return false;
  }
  return true;
}

void SineWave(){
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet,"SINE");
}

void CalStream(){
    StreamSamples<FSRCalSample_t,CAL_TXBUFFERSIZE>(FSRCal::buffer,packet,"FSR");
}

void Stream(){
  if (ErisAscii::describing()){
    // DESC: each feature emits its own TEXT packet, so no framing here.
    for (uint8_t i=0;i<Nfunctions;i++) (*streamfnc[i])();
    return;
  }

  if (ErisAscii::isAscii()){
    // One text line per tick, no COBS framing.
    ErisAscii::frameBegin();
    for (uint8_t i=0;i<Nfunctions;i++) (*streamfnc[i])();
    ErisAscii::frameEnd();
    return;
  }

  packet.start(Packet::PacketType::DATA);
  for (uint8_t i=0;i<Nfunctions;i++) (*streamfnc[i])();
  packet.send();
}

}
