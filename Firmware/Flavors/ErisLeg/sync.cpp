#include "Eris.h"
#include "sync.h"
#include "configuration.h"
#include "serialcommand.h"

#include <IntervalTimer.h>

//#include <FeatureExtractor.h>

namespace Sync{
  

IntervalTimer timer0; // Timer for ADC

static const uint8_t pin_channels[FSR_NUMCHANNELS]={PIN_FSR};

//Buffer for readings 
ErisBuffer<uint8_tSample_t> buffer ;

// Indices and flags

static void ISR_SYNC_CHANGE(){  
  chSysLockFromISR();
  float timestamp = ((float)(micros() - SerialCom::startTime))/1.0e3;   
  uint8_tSample_t thisSample;
  thisSample.timestamp=timestamp;    
  thisSample.value=digitalRead(PIN_SYNC);     
  buffer.append(thisSample);    
  chSysUnlockFromISR();  
}

void start(void){   
    pinMode(PIN_SYNC,INPUT_PULLDOWN);
    buffer.init();                        
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    eriscommon::println("SYNCREADY");
    attachInterrupt(PIN_SYNC,ISR_SYNC_CHANGE,CHANGE);
 }
}
