#include "Eris.h"
#include "emg.h"
#include "sdcard.h"
#include "configuration.h"

#include <ADC.h>
#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace EMG{
  
thread_t *extractFeaturesEMG = NULL;
thread_t *readAnalog = NULL;

IntervalTimer timer0; // Timer for ADC
ADC *adc = new ADC(); // adc object

static const uint8_t pin_channels[NUMEMGCHANNELS]={PIN_CH0};


//Buffer for readings 
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMEMGCHANNELS+1];

static float emgTimestamp;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;

// Indices and flags

static void ISR_NewSample(){  
	chSysLockFromISR();
  emgTimestamp = ((float)(micros() - SDCard::startTime))/1e6;  

  //Sample every analog channel
  for(uint8_t chan = 0; chan < NUMEMGCHANNELS+1; chan++) {
       //*********** TO TEST MISSING SAMPLES ***************//
      // To test missing samples:
      /*
      test=test+1;
      if (test>10000){
        test=0;
      }
      v=test;       
      */
       //*********** TO TEST MISSING SAMPLES ***************//
    
      //Preview data Serial.println(v);      
      #if SDCARD
        //SDCard::addEMG(v,emgTimestamp,chan);
      #endif 
      if (chan == 0) {
        buffer[0]->append(emgTimestamp); 
      }
      //Highly experimental add also to the buffer
      else {
        int value=analogRead(pin_channels[chan]);
        float v=value*(3.3/1024);
        buffer[chan]->append(v); 
      }    
  }

  chSysUnlockFromISR();  
}

void start(void){ 

     for(uint8_t chan = 0; chan < NUMEMGCHANNELS+1; chan++) {               
        buffer[chan]=new ErisBuffer<float>;       
        buffer[chan]->init();                      
     }
     
    chBSemObjectInit(&xsamplesSemaphore,true);    
    
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    adc->setAveraging(4); // set number of averages
    adc->setResolution(10); // set bits of resolution 
    timer0.begin(ISR_NewSample, EMG_PERIOD_US);   	
	}
}
