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
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMEMGCHANNELS];

static float emgTimestamp;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;

// Indices and flags

static void ISR_NewSample(){  
	chSysLockFromISR();
  emgTimestamp = ((float)(micros() - SDCard::startTime))/1000.0;  

  //Sample every analog channel
  for(uint8_t chan = 0; chan < NUMEMGCHANNELS; chan++) {
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
      int value=analogRead(pin_channels[chan-1]);
      float v=value*(3.3/1024);
      

      //Adds timestamps to first buffer, treated like an extra channel
      if (chan == 0) {
        #if SDCARD
        SDCard::addEMG(emgTimestamp,emgTimestamp,chan);
        #endif 
        buffer[chan]->append(emgTimestamp);  
      } 
      else {    
        #if SDCARD
        SDCard::addEMG(v,emgTimestamp,chan);
        #endif   
        buffer[chan]->append(v); 
      }
  }

  chSysUnlockFromISR();  
}

void start(void){ 

     for(uint8_t chan = 0; chan < NUMEMGCHANNELS; chan++) {               
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
