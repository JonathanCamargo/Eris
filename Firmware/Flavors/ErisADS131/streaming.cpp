#include "Eris.h"

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

#define DEBUG 1
bool AddFunction(char * arg){
  #ifdef DEBUG
    Serial.print("Function requested: ");
    Serial.println(arg);
  #endif
  if (!strncmp("EMG",arg,MAXSTRCMP)){
    Serial.println("EMG selected");
    streamfnc[Nfunctions]=&EMG;
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

void EMG(){
    StreamSamples<EMGSample_t,TXBUFFERSIZE>(ADS131::buffer,packet);  
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
