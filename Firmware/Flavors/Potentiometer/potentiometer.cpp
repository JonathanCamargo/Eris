#include "Eris.h"
#include "potentiometer.h"

namespace Potentiometer{
	
eris_thread_ref_t getSample = NULL;

//Buffer for readings 
ErisBuffer<floatSample_t> buffer;

  
ERIS_THREAD_WA(waGetSample_T, 128);
ERIS_THREAD_FUNC(GetSample_T) {  
  static long idx=0;
  while(1){
    
    float timestamp = ((float)(micros() - t0))/1.0e3;      
    float value=POTENTIOMETER_GAIN*(3.3*analogRead(PIN_POTENTIOMETER)/1024)+POTENTIOMETER_OFFSET;
    
    floatSample_t thisSample;
    thisSample.timestamp=timestamp;
    thisSample.value=value;    
    buffer.append(thisSample); 
       
    eris_sleep_ms(POTENTIOMETER_PERIOD_MS);    
  }
}
	
void start(void){        
    buffer.init();
    // create tasks at priority lowest priority
    getSample=eris_thread_create(waGetSample_T, 128,NORMALPRIO+1, GetSample_T, NULL);
}
	
	
}
