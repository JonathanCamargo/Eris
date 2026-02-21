#include "Eris.h"
#include "fsr.h"
#include "sdcard.h"
#include "configuration.h"
#include "serialcommand.h"

#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace FSR{
  
eris_thread_ref_treadAnalog = NULL;

IntervalTimer timer0; // Timer for ADC

static const uint8_t pin_channels[FSR_NUMCHANNELS]={PIN_FSR};

//Buffer for readings 
ErisBuffer<FSRSample_t> buffer ;

// Semaphore to feature extractor
static eris_binary_sem_t xsamplesSemaphore;

// Indices and flags

static void ISR_NewSample(){  
  ERIS_CRITICAL_ENTER();
  float timestamp = ((float)(micros() - SerialCom::startTime))/1.0e3;  
  FSRSample_t thisSample;
  thisSample.timestamp=timestamp;  
  //Sample every analog channel
  for(uint8_t chan = 0; chan < FSR_NUMCHANNELS; chan++) {                
      int value=analogRead(pin_channels[chan-1]);
      float v=value*(3.3/1024);    
      thisSample.ch[chan]=v;      
  }  
  buffer.append(thisSample);    
  #if SDCARD
      SDCard::addFSR(thisSample);
  #endif 
  ERIS_CRITICAL_EXIT();  
}

void start(void){   
    buffer.init();                    
    eris_bsem_init(&xsamplesSemaphore,true);        
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    analogReadAveraging(4); // set number of averages
    analogReadRes(10); // set bits of resolution   
    timer0.begin(ISR_NewSample, FSR_PERIOD_US);          
  }
}
