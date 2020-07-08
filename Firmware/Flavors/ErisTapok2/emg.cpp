#include "Eris.h"
#include "emg.h"
#include "sdcard.h"
#include "configuration.h"

#include <tapok_4.h>

#include <FeatureExtractor.h>

namespace EMG{
  
thread_t *extractFeaturesEMG = NULL;
thread_t *readAnalog = NULL;

Tapok emg(EMG_GAIN); //Tapok object to read the data

//Buffer for readings 
ErisBuffer<EMGSample_t> buffer;
ErisBuffer<EMGSample_t> emgbuffer;
ErisBuffer<EMGSample_t> emgbuffer2;
ErisBuffer<EMGSample_t> emgbuffer3;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;

// Indices and flags
static void ISR_NewSample(){  
  chSysLockFromISR();
  digitalWrite(PIN_LED,HIGH);
  float timestamp = ((float)(micros() - SDCard::startTime))/1000.0;  
  //Sample every channel
  EMGSample_t thisSample;
  thisSample.timestamp=timestamp; 
  for(uint8_t chan = 0; chan < EMG_NUMCHANNELS; chan++) {
      int value=emg.lastSample[chan]-32800;
      float v=value*(3.3/1024); //TODO define a good scaling ratio               
      //Preview data Serial.println(v);           
      thisSample.ch[chan]=v;      
  }  
  buffer.append(thisSample);
  #if SDCARD
    //SDCard::addEMG(thisSample);
    SDCard::emgbuffer.append(thisSample);
    emgbuffer.append(thisSample);
    emgbuffer2.append(thisSample);
    emgbuffer3.append(thisSample);
  #endif 
  digitalWrite(PIN_LED,LOW);
  chSysUnlockFromISR();  
}

void start(void){ 
    CANTAPOK.begin(1000000); 

    // Start ErisBuffers        
    buffer.init();                          
    emgbuffer.init();
    emgbuffer2.init();
    emgbuffer3.init();
    //Start samples semaphore for feature extraction
    chBSemObjectInit(&xsamplesSemaphore,true);    
    
    // Initialize interrupt to take the samples      
    emg.connect();     
    emg.start();  //Start data stream
    delay(100);
    emg.subscribeFun(ISR_NewSample);  
  }
}
