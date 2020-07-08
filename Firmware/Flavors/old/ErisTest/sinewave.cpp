#include "Eris.h"
#include "sinewave.h"


#if SDCARD
#include "sdcard.h"
#endif

#if FEATURES
#include <FeatureExtractor.h>

#endif

namespace SineWave{
	
thread_t *generateWave = NULL;

//Buffer for readings 
ErisBuffer<float> buffer;

long idx;   

static const float FREQ=1;
static float sineTimestamp;


static THD_WORKING_AREA(waGenerateWave_T, 128);
static THD_FUNCTION(GenerateWave_T, arg) {  
  while(1){
    if (idx>=10000){
	  idx=0;
	  }
    else{
    idx=idx+1;
    }
    float value=3.3*sin(2*M_PI*FREQ*idx/10000);          
    sineTimestamp = ((float)(micros() - SDCard::startTime))/1000.0;  


    buffer.append(value);

    // Write data to sdcard (implemented in sdcard.cpp)
    #if SDCARD
        SDCard::addSine(sineTimestamp,value);
    #endif 
       
    chThdSleepMilliseconds(10);    
  }
}

	
	void start(void){    
    idx=0;    
    buffer.init();
    // create tasks at priority lowest priority
    generateWave=chThdCreateStatic(waGenerateWave_T, sizeof(waGenerateWave_T),NORMALPRIO, GenerateWave_T, NULL);
	}
	
	
	
	
	
	
	
	
	
}
