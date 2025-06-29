#include "Eris.h"
#include "biom.h"

#include "features.h"
#include "configuration.h"
#include "gait.h"
#include "serialcommand.h"

#include <IntervalTimer.h>

namespace Biom{

  /////////////////////////////////////////////////////////
  IntervalTimer timer0; // Timer for ADC
  
  static uint8_t pin_channels[BIOM_NUMCHANNELS] = {PIN_BIOM};
  
  //Buffer for readings 
  ErisBuffer<BiomSample_t> buffer;
  
  // Semaphore to feature extractor
  static binary_semaphore_t xsamplesSemaphore;
  
  // Indices and flags
  long idx = 0;

  FeatureExtractorPtr extractor[BIOM_NUMCHANNELS];
  
  ////////////////////////////////////////////////////////

  static void ISR_NewSample(){
  	chSysLockFromISR();
    
    chBSemSignalI(&xsamplesSemaphore);
    
    BiomSample_t thisSample;
      
    thisSample.timestamp = ((float)(micros() - t0))/1000.0;
    
    for(uint8_t chan = 0; chan < BIOM_NUMCHANNELS; chan++) {
      
      int value=analogRead(pin_channels[chan]);
      float v = (value*(3.3/1024)*(320.0/220.0))-2.0;
  
      //v = 2.0*sin(2*(3.14)*(idx)/750.0);
      v = idx;
      
      #if FEATURES
        //Add each channel's value to its respective FeatureExtractor object
        extractor[chan]->newSample(v);
      #endif
      
      thisSample.ch[chan] = v;
    }
    buffer.append(thisSample);
    idx++;
  
    //if(idx > 100) {
    //  idx = -100;
    //}
    
    chSysUnlockFromISR();
  }

  void RegisterExtractors(FeaturesHelper * featuresHelper){
  //Link the feature extractors from this module to a features helper    
      for (uint8_t i=0;i<BIOM_NUMCHANNELS;i++){      
           char name[5];
           sprintf(name,"BIOM%d",i);
        #if FEATURES
           featuresHelper->RegisterExtractor(extractor[i],BIOM_FREQUENCY_HZ,name);
        #endif
      }
  }
  
  void start(void){

    buffer.init();
    chBSemObjectInit(&xsamplesSemaphore,true);
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    analogReadAveraging(4); // set number of averages
    analogReadRes(10); // set bits of resolution

    // Initialize extractors 
    for (uint8_t i=0;i<BIOM_NUMCHANNELS;i++){
      extractor[i]=new FeatureExtractor();
    }  
    
    timer0.begin(ISR_NewSample, BIOM_PERIOD_US);
	}
}
