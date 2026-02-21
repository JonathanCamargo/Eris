#include "Eris.h"
#include "sinewave.h"


#if SDCARD
#include "sdcard.h"
#endif

namespace SineWave{
	
eris_thread_ref_tgenerateWave = NULL;

//Buffer for readings 
ErisBuffer<floatSample_t> buffer;

long idx;   

static const float FREQ=1;
  
ERIS_THREAD_WA(waGenerateWave_T, 128);
ERIS_THREAD_FUNC(GenerateWave_T) {  
  while(1){
    if (idx>=10000){
	  idx=0;
	  }
    else{
    idx=idx+1;
    }

    float timestamp = ((float)(micros() - t0))/1.0e3;  
    float value=3.3*sin(2*M_PI*FREQ*idx/10000);             
    
    floatSample_t thisSample;
    
    thisSample.timestamp=timestamp;
    thisSample.value=value;
    buffer.append(thisSample);

    eris_sleep_ms(10);    
  }
}

	
	void start(void){    
    idx=0;    
    buffer.init();
    // create tasks at priority lowest priority
    generateWave=eris_thread_create(waGenerateWave_T, sizeof(waGenerateWave_T),NORMALPRIO, GenerateWave_T, NULL);
	}
	
	
	
	
	
	
	
	
	
}
