#include "Eris.h"
#include "streaming.h"

#include "imu.h"
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
    // IMU::buffer is an array now (ERIS_SENSOR_MULTI), so the old per-device
    // switch is just an index. Out-of-range falls back to device 0.
    if (imuidx < 0 || imuidx >= IMU_COUNT) imuidx = 0;

    // The feature label keeps the two devices apart in the Serial Plotter
    // legend: "IMU_0.ax" vs "IMU_1.ax". Ignored on the binary path.
    StreamSamples<IMUSample_t,IMU_TXBUFFERSIZE>(IMU::buffer[imuidx],packet,
                                                imuidx==0 ? "IMU_0" : "IMU_1");
    //StreamSamplesMemoryEfficient<IMUSample_t,IMU_TXBUFFERSIZE>(*imubuffer,packet,imusamples);
    
}

 void SineWave(){
    //StreamSamplesMemoryEfficient<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet,floatsamples);
    StreamSamples<floatSample_t,TXBUFFERSIZE>(SineWave::buffer,packet,"SINE");
}

void Stream(){
  //Fetch data from desired buffers and send via serial
  if (ErisAscii::describing()){
    // DESC: each feature emits its own TEXT packet, so no framing here.
    for (uint8_t i=0;i<Nfunctions;i++){
       (*streamfnc[i])();
    }
    return;
  }

  if (ErisAscii::isAscii()){
    // One text line per tick, no COBS framing.
    ErisAscii::frameBegin();
    for (uint8_t i=0;i<Nfunctions;i++){
       (*streamfnc[i])();
    }
    ErisAscii::frameEnd();
    return;
  }

  packet.start(Packet::PacketType::DATA); 
  for (uint8_t i=0;i<Nfunctions;i++){
     (*streamfnc[i])();
  }   
  packet.send(); 
}

}
