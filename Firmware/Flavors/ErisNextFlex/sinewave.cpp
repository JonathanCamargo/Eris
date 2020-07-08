#include "Eris.h"
#include "sinewave.h"
#include "sdcard.h"

namespace SineWave{
  
thread_t *generateWave = NULL;

//Buffer for readings 
ErisBuffer<floatSample_t> buffer;



static const float FREQ=1;
  
static THD_WORKING_AREA(waGenerateWave_T, 128);
static THD_FUNCTION(GenerateWave_T, arg) {  
  static long idx=0;
  while(1){
    if (idx>=10000){
    idx=0;
    }
    else{
    idx=idx+1;
    }

    
    float timestamp = ((float)(micros() - t0))/1.0e3;  
    
    float value=3.3*sin(2*M_PI*FREQ*((float)idx)/1.0e4);          
    //float value=timestamp;
    
    floatSample_t thisSample;
    thisSample.timestamp=timestamp;
    thisSample.value=value;    
    buffer.append(thisSample);
  
    // Write data to sdcard (implemented in sdcard.cpp)
    #if SDCARD
        SDCard::sinebuffer.append(thisSample);
    #endif 
       
    chThdSleepMilliseconds(10);    
  }
}

  
  void start(void){        
    buffer.init();
    // create tasks at priority lowest priority
    generateWave=chThdCreateStatic(waGenerateWave_T, sizeof(waGenerateWave_T),NORMALPRIO+1, GenerateWave_T, NULL);
  }
  
  
  
  
  
  
  
  
  
}
