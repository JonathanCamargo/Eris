#include "Eris.h"
#include "sinewave.h"

namespace SineWave{
	
eris_thread_ref_t generateWave = NULL;

//Buffer for readings 
ErisBuffer<floatSample_t> buffer;
ErisBuffer<multiSample_t> buffer2;


static const float FREQ=1;
  
ERIS_THREAD_WA(waGenerateWave_T, 128);
ERIS_THREAD_FUNC(GenerateWave_T) {  
  static long idx=0;
  static floatSample_t thisSample;
  static multiSample_t mSample;
  while(1){
    if (idx>=10000){
	  idx=0;
	  }
    else{
    idx=idx+1;
    }
    
    float timestamp = ((float)(micros() - t0))/1.0e3;  
    
    float value=3.3*sin(2*M_PI*FREQ*((float)idx)/1.0e4);          
    
    
    thisSample.timestamp=timestamp;
    thisSample.value=value;    
    buffer.append(thisSample); 

    
    mSample.timestamp=timestamp;

    for (uint8_t i=0;i<NUMCHANNELS;i++){
      mSample.value[i]=value*(1+i/10);
    }
    buffer2.append(mSample);
   
    eris_sleep_ms(10);
  }
}

	
	void start(void){        
    buffer.init();
    buffer2.init();
    // create tasks at priority lowest priority
    generateWave=eris_thread_create(waGenerateWave_T, 128, NORMALPRIO+1, GenerateWave_T, NULL);
	}
	
	
}
