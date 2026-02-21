#include "Eris.h"
#include "streaming.h"

#include "biom.h"
#include "features.h"
#include "sinewave.h"
#include "gait.h"

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
  else if (!strncmp("BIOM",arg,MAXSTRCMP)){
      Serial.println("Biom selected");
    streamfnc[Nfunctions]=&Biom;
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("Gait",arg,MAXSTRCMP)){
      Serial.println("Gait selected");
    streamfnc[Nfunctions]=&Gait;
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

void Biom(){
  //Transmit Biometrics buffer
  StreamSamples<BiomSample_t,BIOM_TXBUFFERSIZE>(Biom::buffer,packet);
}

void Gait() {
  packet.append((uint8_t *)&Gait::currGait, sizeof(float));
}


void Stream(){
  if(Nfunctions > 0) {
    packet.start(Packet::PacketType::DATA);
    //Fetch data from desired buffers and send via serial
    for (uint8_t i=0;i<Nfunctions;i++){
      (*streamfnc[i])();
    }
     packet.send();
  }
}

}
