#include "Eris.h"
#include "emg.h"
#include "configuration.h"

#include <tapok.h>

namespace EMG{
  
thread_t *readAnalog = NULL;

Tapok emg(2,CANTAPOK); //Tapok object to read the data

//Buffer for readings 
ErisBuffer<EMGSample_t> buffer;

static void ISR_NewSample(){  
	chSysLockFromISR();
  float timestamp = ((float)(micros() - t0))/1.0e3;  

  EMGSample_t thisSample;
  thisSample.timestamp=timestamp;
  
  //Sample every analog channel
  for(uint8_t chan = 0; chan < EMG_NUMCHANNELS; chan++) {
      int value=emg.lastSample[chan]-32800;
      float v=value*(3.3/1024); //TODO define a good scaling ratio            
      thisSample.ch[chan]=v;      
  }  
  buffer.append(thisSample); 
  chSysUnlockFromISR();  
}

void start(void){ 
    CANTAPOK.begin(1000000); 

    // Start ErisBuffers
    buffer.init();                           
    // Initialize interrupt to take the samples 
    emg.subscribeFun(&ISR_NewSample);    
    emg.connect();  
    delay(100);
    emg.start();  //Start data stream
	}
}
