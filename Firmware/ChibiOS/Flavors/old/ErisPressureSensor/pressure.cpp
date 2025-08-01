#include "Eris.h"
#include "pressure.h"

#include <eriscommon.h>

#if SDCARD
#include "sdcard.h"
#endif

#if FEATURES
#include <FeatureExtractor.h>

#endif

namespace Pressure{
	
thread_t *generateWave = NULL;

#if FEATURES
FeatureExtractor extractor(100,25);
#endif

//Buffer for readings 
ErisBuffer<float> buffer;
ErisBuffer<float> rmsbuffer;
 
long idx;   

static const float FREQ=1;
  
static THD_WORKING_AREA(waGenerateWave_T, 128);
static THD_FUNCTION(GenerateWave_T, arg) {  
  while(1){
    if (idx>=10000){
	  idx=0;
	  }
    else{
    idx=idx+1;
    }
    float value=float(analogRead(PIN_ANALOG))/1024.0;          
    
    buffer.append(value);

    // Write data to sdcard (implemented in sdcard.cpp)
    #if SDCARD
        SDCard::addSine(value);
    #endif 
    
    // Send to the feature extractor
    #if FEATURES
      if (extractor.newSample(value)){
        //Serial.println(extractor.rms());
        float rms=extractor.rms();
        rmsbuffer.append(rms);
        extractor.clear();        
      }
    #endif
    
    chThdSleepMilliseconds(10);    
  }
}

	
	void start(void){    
    idx=0;    
    buffer.init();
    #if FEATURES
    rmsbuffer.init();
    #endif
    // create tasks at priority lowest priority
    generateWave=chThdCreateStatic(waGenerateWave_T, sizeof(waGenerateWave_T),NORMALPRIO, GenerateWave_T, NULL);
	}
	
	
	
	
	
	
	
	
	
}
