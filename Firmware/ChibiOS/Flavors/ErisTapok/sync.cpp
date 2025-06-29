#include "Eris.h"
#include "sync.h"
#include "configuration.h"

#include <IntervalTimer.h>

namespace Sync{
 
IntervalTimer timer0; // Timer for ADC

//Buffer for readings 
ErisBuffer<uint8_tSample_t> buffer;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;

// Indices and flags

static long test=0;
static void ISR_NewSample(){  
  chSysLockFromISR();

  uint8_tSample_t thisSample;
  
  float timestamp = ((float)(micros() - t0))/1000.0;  
  bool value=!digitalRead(PIN_SYNC);  
  digitalWrite(PIN_SYNC,value);  
  thisSample.timestamp=timestamp;
  thisSample.value=value;   
  buffer.append(thisSample); 
  
  chSysUnlockFromISR();  
}

void start(void){ 
    pinMode(PIN_SYNC,OUTPUT);
    digitalWrite(PIN_SYNC,HIGH);
    buffer.init();       
                
    timer0.begin(ISR_NewSample, SYNC_FREQUENCY_US);          
  }
}
