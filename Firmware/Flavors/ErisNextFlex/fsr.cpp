#include "Eris.h"
#include "fsr.h"
#include "sdcard.h"
#include "configuration.h"
#include "serialcommand.h"

#include <IntervalTimer.h>

namespace FSR{
  
eris_thread_ref_t newSample = NULL;


static const uint8_t pin_channels[FSR_NUMCHANNELS]={PIN_FSR};

//Buffer for readings 
ErisBuffer<FSRSample_t> buffer ;

// Semaphore to feature extractor
static eris_binary_sem_t xsamplesSemaphore;

// Indices and flags

 
ERIS_THREAD_WA(waNewSample_T, 128);
ERIS_THREAD_FUNC(NewSample_T) {    
  while(1){
    float timestamp = ((float)(micros() - t0))/1.0e3;     
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
     SDCard::fsrbuffer.append(thisSample);
  #endif  
    eris_sleep_ms(10);    
  }
}

void start(void){   
    buffer.init();                    
    eris_bsem_init(&xsamplesSemaphore,true);        
  //Configure ADC for FSR
    analogReadAveraging(4); // set number of averages
    analogReadRes(10); // set bits of resolution   
    
  // create tasks at priority lowest priority
    newSample=eris_thread_create(waNewSample_T, sizeof(waNewSample_T),NORMALPRIO+1, NewSample_T, NULL);
  }
}
