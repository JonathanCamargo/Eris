#include "Eris.h"
#include "biom.h"
#include "configuration.h"
#include "serialcommand.h"

#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace Sync{
  

IntervalTimer timer0; // Timer for ADC

static const uint8_t pin_channels[BIOM_NUMCHANNELS]={PIN_BIOM};

//Buffer for readings 
ErisBuffer<boolSample_t> buffer ;

// Indices and flags

static void ISR_SYNC_CHANGE(){  
  chSysLockFromISR();
  float timestamp = ((float)(micros() - SerialCom::startTime))/1.0e3;   
  boolSample_t thisSample;
  thisSample.timestamp=timestamp;    
  thisSample.value=digitalRead(PIN_SYNC);     
  buffer.append(thisSample);    
  #if SDCARD
      SDCard::syncbuffer.append(thisSample);
  #endif
  chSysUnlockFromISR();  
}

void start(void){   
    pinMode(PIN_SYNC,INPUT_PULLUP);
    buffer.init();                        
    // Timer interrupt to take the ADC samples        
    //Set up ADC
    Serial.println("SYNCREADY");
    attachInterrupt(PIN_SYNC,ISR_SYNC_CHANGE,CHANGE);
 }
}
