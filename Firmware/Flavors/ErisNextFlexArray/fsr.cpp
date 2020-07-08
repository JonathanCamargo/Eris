#include "Eris.h"
#include "fsr.h"
#include "sdcard.h"
#include "configuration.h"
#include "serialcommand.h"

#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace FSR{
  
thread_t *readAnalog = NULL;


IntervalTimer timer0; // Timer for ADC

static const uint8_t pin_channels[NUMFSRCHANNELS]={PIN_FSR};


//Buffer for readings 
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMFSRCHANNELS];
ErisBuffer<float> buffer_timestamp;


static float fsrTimestamp;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;

// Indices and flags

static long test=0;
static void ISR_NewSample(){  
  chSysLockFromISR();
  fsrTimestamp = ((float)(micros() - SerialCom::startTime))/1.0e6;  

  //Sample every analog channel
  for(uint8_t chan = 0; chan < NUMFSRCHANNELS+1; chan++) {    
      
      if(chan == 0) {
        buffer_timestamp.append(fsrTimestamp);
      }       
      int value=analogRead(pin_channels[chan-1]);
      float v=value*(3.3/1024);    
      buffer[chan]->append(v); 
      #if SDCARD
         SDCard::addFSR(v,fsrTimestamp,chan);
      #endif
  }  

  chSysUnlockFromISR();  
}

void start(void){ 

     for(uint8_t chan = 0; chan < NUMFSRCHANNELS+1; chan++) {               
        buffer[chan]=new ErisBuffer<float>;       
        buffer[chan]->init();  
        buffer_timestamp.init();                 
     }
     
    chBSemObjectInit(&xsamplesSemaphore,true);    
    
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    analogReadAveraging(4); // set number of averages
    analogReadRes(10); // set bits of resolution   
    timer0.begin(ISR_NewSample, FSR_PERIOD_US);          
  }
}
