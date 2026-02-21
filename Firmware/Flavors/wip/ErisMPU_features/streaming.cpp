#include "Eris.h"
#include "streaming.h"

#include "imu.h"
#include "fsr.h"
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
  else if (!strncmp("FSR",arg,MAXSTRCMP)){
    Serial.println("FSR selected");
    streamfnc[Nfunctions]=&FSR;
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
  else if (!strncmp("IMU_2",arg,MAXSTRCMP)){
    #if DEBUG
      Serial.println("IMU_2 selected");
    #endif
    streamfnc[Nfunctions]=&IMU_2;
    Nfunctions=Nfunctions+1;
  }    
  else if (!strncmp("IMU_3",arg,MAXSTRCMP)){
    #if DEBUG
      Serial.println("IMU_3 selected");
    #endif
    streamfnc[Nfunctions]=&IMU_3;
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


 void IMU_0(){
  IMU_byIdx(0);
}
 void IMU_1(){
  IMU_byIdx(1);
}
 void IMU_2(){
  IMU_byIdx(2);
}
 void IMU_3(){
  IMU_byIdx(3);
}

 void IMU_byIdx(int imuidx){
  ErisBuffer<IMUSample_t> * imubuffer;  
    switch (imuidx){
      case 0: 
        imubuffer=&IMU::bufferTrunk;
        break;
      case 1:
        imubuffer=&IMU::bufferThigh;
        break;
      case 2:
        imubuffer=&IMU::bufferShank;
        break;
      case 3:
        imubuffer=&IMU::bufferFoot;
        break;
      default:
        imubuffer=&IMU::bufferTrunk;
        break;
  
    }         

    StreamSamples<IMUSample_t,IMU_TXBUFFERSIZE>(*imubuffer,packet);
    //StreamSamplesMemoryEfficient<IMUSample_t,IMU_TXBUFFERSIZE>(*imubuffer,packet,imusamples);
    
}

 void SineWave(){
    //StreamSamplesMemoryEfficient<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet,floatsamples);
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet);
}
 void FSR(){    
    //StreamSamplesMemoryEfficient<FSRSample_t,FSR_TXBUFFERSIZE>(FSR::buffer,packet,fsrsamples);
    StreamSamples<FSRSample_t,FSR_TXBUFFERSIZE>(FSR::buffer,packet);
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
