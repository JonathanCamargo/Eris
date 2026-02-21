#include "Eris.h"
#include "potentiometer.h"

namespace Potentiometer{
	
thread_t *getSample = NULL;

//Buffer for readings 
ErisBuffer<floatSample_t> buffer;

  
static THD_WORKING_AREA(waGetSample_T, 128);
static THD_FUNCTION(GetSample_T, arg) {  
  static long idx=0;
  while(1){
    
    float timestamp = ((float)(micros() - t0))/1.0e3;      
    float value=POTENTIOMETER_GAIN*(3.3*analogRead(PIN_POTENTIOMETER)/1024)+POTENTIOMETER_OFFSET;
    
    floatSample_t thisSample;
    thisSample.timestamp=timestamp;
    thisSample.value=value;    
    buffer.append(thisSample); 
       
    chThdSleepMilliseconds(POTENTIOMETER_PERIOD_MS);    
  }
}
	
void start(void){        
    buffer.init();
    // create tasks at priority lowest priority
    getSample=chThdCreateStatic(waGetSample_T, sizeof(waGetSample_T),NORMALPRIO+1, GetSample_T, NULL);
}
	
	
}
