#include "Eris.h"
#include "fsr.h"

namespace FSR{
  
eris_thread_ref_tsampleFSR = NULL;

//Buffer for readings 
ErisBuffer<FSRSample_t> buffer;

 
ERIS_THREAD_WA(waSampleFSR_T, 128);
ERIS_THREAD_FUNC(SampleFSR_T) {    
  while(1){
    
    float timestamp = ((float)(micros() - t0))/1.0e3;  
    
    float value=3.3*analogRead(PIN_FSR)/1024.0;
    //float value=timestamp;    
    //Serial.println(value);
    
    FSRSample_t thisSample;
    thisSample.timestamp=timestamp;
    thisSample.ch[0]=value;    
    buffer.append(thisSample);      
       
    eris_sleep_ms(FSR_PERIOD_US/1000);    
  }
}

  
  void start(void){        
    buffer.init();
    // create tasks at priority lowest priority
    sampleFSR=eris_thread_create(waSampleFSR_T, sizeof(waSampleFSR_T),NORMALPRIO+1, SampleFSR_T, NULL);
  }
  
  
  
  
  
  
  
  
  
}
