#include "Eris.h"
#include "fsr.h"

namespace FSR{
  
thread_t *sampleFSR = NULL;

//Buffer for readings 
ErisBuffer<FSRSample_t> buffer;

 
static THD_WORKING_AREA(waSampleFSR_T, 128);
static THD_FUNCTION(SampleFSR_T, arg) {  
  static long idx=0;
  while(1){
    
    float timestamp = ((float)(micros() - t0))/1.0e3;  
    
    float value=3.3*analogRead(PIN_FSR)/1024.0;
    //float value=timestamp;    
    //Serial.println(value);
    
    FSRSample_t thisSample;
    thisSample.timestamp=timestamp;
    thisSample.ch[0]=value;    
    buffer.append(thisSample);      
       
    chThdSleepMilliseconds(FSR_PERIOD_US/1000);    
  }
}

  
  void start(void){        
    buffer.init();
    // create tasks at priority lowest priority
    sampleFSR=chThdCreateStatic(waSampleFSR_T, sizeof(waSampleFSR_T),NORMALPRIO+1, SampleFSR_T, NULL);
  }
  
  
  
  
  
  
  
  
  
}
