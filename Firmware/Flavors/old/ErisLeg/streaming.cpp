#include "Eris.h"
#include "streaming.h"

#include "sinewave.h"

#include <string.h>

namespace Streaming{
#define MAXSTRCMP 4
#define MAXFNC 10
static void (*streamfnc[MAXFNC])();
uint8_t Nfunctions=0;


void ClearFunctions(){
  Nfunctions=0;
}

#define DEBUG 1
bool AddFunction(char * arg){
  #ifdef DEBUG
    Serial.print("Function requested: ");
    Serial.println(arg);
  #endif
  if (!strncmp("SineWave",arg,MAXSTRCMP)){
    #ifdef DEBUG
      Serial.println("SineWave selected");
    #endif
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }  
  else if (!strncmp("RMS",arg,MAXSTRCMP)){
    #ifdef DEBUG
      Serial.println("EMG rms selected");
    #endif  
    streamfnc[Nfunctions]=&rms;    
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
  //Transmit SineWave buffer
  float dest1[TXBUFFERSIZE];
  uint8_t num=SineWave::buffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
  Serial.write((char *)dest1,TXBUFFERSIZE*sizeof(float));
}

void rms(){
  //Transmit emg rms buffer
  float dest1[TXBUFFERSIZE];
  uint8_t num=EMG::rmsbuffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
  Serial.write((char *)dest1,TXBUFFERSIZE*sizeof(float));
}



void Stream(){
  //Fetch data from desired buffers and send via serial
  for (uint8_t i=0;i<Nfunctions;i++){
    (*streamfnc[i])();
  }
}

}
