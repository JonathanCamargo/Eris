#include "Eris.h"
#include "streaming.h"

#include "imu.h"
#include "sinewave.h"

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
    Serial.print("Function requested: ");
    Serial.println(arg);
  #endif
  if (!strncmp("SINE",arg,MAXSTRCMP)){
    #if DEBUG
      Serial.println("SineWave selected");
    #endif
    streamfnc[Nfunctions]=&SineWave;
    Nfunctions=Nfunctions+1;
  }    
  else if (!strncmp("IMU_0",arg,MAXSTRCMP)){
    #if DEBUG
      Serial.println("IMU_0 selected");
    #endif
    streamfnc[Nfunctions]=&IMU_0;
    Nfunctions=Nfunctions+1;
  }    
  else if (!strncmp("IMU_1",arg,MAXSTRCMP)){
    #if DEBUG
      Serial.println("IMU_1 selected");
    #endif
    streamfnc[Nfunctions]=&IMU_1;
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


 void IMU_0(){
  IMU_byIdx(0);
}
 void IMU_1(){
  IMU_byIdx(1);
}

 void IMU_byIdx(int imuidx){
  ErisBuffer<IMUSample_t> * imubuffer;
    switch (imuidx){
      case 0:
        imubuffer=&IMU::buffer0;
        break;
      case 1:
        imubuffer=&IMU::buffer1;
        break;
      default:
        imubuffer=&IMU::buffer0;
        break;
    }

    StreamSamples<IMUSample_t,IMU_TXBUFFERSIZE>(*imubuffer,packet);
    //StreamSamplesMemoryEfficient<IMUSample_t,IMU_TXBUFFERSIZE>(*imubuffer,packet,imusamples);
    
}

 void SineWave(){
    //StreamSamplesMemoryEfficient<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet,floatsamples);
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
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
