#include "Eris.h"
#include "streaming.h"

#include "eeg.h"
#include "fsr.h"
#include "sinewave.h"

#include <string.h>
#include <packet.h>

namespace Streaming{

#define MAXSTRCMP 5
#define MAXFNC 10

static void (*streamfnc[MAXFNC])();
uint8_t Nfunctions=0;

Packet packet;

static float dest1[TXBUFFERSIZE]; //Global destination buffer to reduce stack

// Forward Declaration of static functions:
static void SineWave();
static void EEG();
static void FSR();

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
  else if (!strncmp("EMG",arg,MAXSTRCMP)){
    #ifdef DEBUG
      Serial.println("EEG selected");
    #endif  
    streamfnc[Nfunctions]=&EEG;    
    Nfunctions=Nfunctions+1;
  }
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    #ifdef DEBUG
      Serial.println("FSR selected");
    #endif  
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

/************************ Custom transmision of data *********************************/
static void SineWave(){
  //Transmit SineWave buffer
  uint8_t num=SineWave::buffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
  packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float));
}

static void EEG(){
  //Transmit EEG buffers  
  for (uint8_t chan=0;chan<NUMEEGCHANNELS;chan++){  
    uint8_t num=EEG::buffer[chan]->FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);     
    for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
      dest1[i]=NAN;    
    }    
    packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float));
}
}

static void FSR(){
  //Transmit FSR buffer
  uint8_t num=FSR::buffer.FetchData(dest1,(char *)"SerialCommand",TXBUFFERSIZE);
  for (int i=num;i<TXBUFFERSIZE;i++){//pad with nans
    dest1[i]=NAN;    
  }
  packet.append((uint8_t *)dest1,TXBUFFERSIZE*sizeof(float));
}
/**************************************************************************************/


void Stream(){
  packet.start(Packet::PacketType::DATA);
  //Fetch data from desired buffers and send via serial
  for (uint8_t i=0;i<Nfunctions;i++){
     (*streamfnc[i])();
  }
  packet.send();
}

}
