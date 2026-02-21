#include "Eris.h"
#include "streaming.h"
#include "packet.h"

#include "sinewave.h"
#include "pressure.h"
#include <string.h>

namespace Streaming{
#define MAXSTRCMP 5
#define MAXFNC 10

static void (*streamfnc[MAXFNC])();
uint8_t Nfunctions=0;

Packet packet;

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
  #if FEATURES  
  else if (!strncmp("SineRMS",arg,MAXSTRCMP)){
    Serial.println("Sinewave rms selected");
    streamfnc[Nfunctions]=&SineWaveRMS;    
    Nfunctions=Nfunctions+1;
  }
  #endif
 else if (!strncmp("P",arg,MAXSTRCMP)){
    Serial.println("Pressure raw data selected");
    streamfnc[Nfunctions]=&Pressure;    
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


void Pressure(){
  //Transmit Pressure buffer
  float dest1[TXBUFFERSIZE];
  uint8_t num=Pressure::buffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
  packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float));
}

#if FEATURES
void SineWaveRMS(){
  //Transmit SineWaveRMS buffer
  float dest1[TXBUFFERSIZE];
  uint8_t num=SineWave::rmsbuffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
  packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float)); 
}
#endif


void Stream(){
  //Fetch data from desired buffers and send via serial
  packet.start(Packet::PacketType::DATA);
  for (uint8_t i=0;i<Nfunctions;i++){
    (*streamfnc[i])();
  }
  packet.send();
}

}
