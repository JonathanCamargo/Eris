#include "Eris.h"
#include "fsr.h"
#include "sdcard.h"
#include "configuration.h"

#include <ADC.h>
#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace FSR{
  
thread_t *readAnalog = NULL;


IntervalTimer timer0; // Timer for ADC
ADC *adc = new ADC(); // adc object

static const uint8_t pin_channels[NUMFSRCHANNELS]={PIN_FSR};


//Buffer for readings 
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMFSRCHANNELS];

static float fsrTimestamp;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;

// Indices and flags

static void ISR_NewSample(){  
  chSysLockFromISR();
  fsrTimestamp = ((float)(micros() - SDCard::startTime))/1e6;  

  //Sample every analog channel
  for(uint8_t chan = 0; chan < NUMFSRCHANNELS; chan++) {
      int value=analogRead(pin_channels[chan]);
      float v=value*(3.3/1024);
      // To test missing samples:
      //test=test+1;
      //if (test>10000){
      //  test=0;
      //}
      // v=test;       
      //*/
      //Preview data Serial.println(v);      
      #if SDCARD
        SDCard::addFSR(v,fsrTimestamp,chan);
      #endif 

      //Highly experimental add also to the buffer
      buffer[chan]->append(v);      
  }  

  chSysUnlockFromISR();  
}

void start(void){ 

     for(uint8_t chan = 0; chan < NUMFSRCHANNELS; chan++) {               
        buffer[chan]=new ErisBuffer<float>;       
        buffer[chan]->init();                      
     }
     
    chBSemObjectInit(&xsamplesSemaphore,true);    
    
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    adc->setAveraging(4); // set number of averages
    adc->setResolution(10); // set bits of resolution   
    timer0.begin(ISR_NewSample, FSR_PERIOD_US);          
  }
}
