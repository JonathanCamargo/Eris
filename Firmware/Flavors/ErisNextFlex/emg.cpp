#include "Eris.h"
#include "emg.h"
#include "sdcard.h"
#include "serialcommand.h"
#include "configuration.h"

#include <TimerOne.h>
#include <ADS1256_4CH.h>

namespace EMG{
  
thread_t *extractFeaturesEMG = NULL;
thread_t *readSensor1 = NULL;

//Buffer for readings 
ErisBuffer<EMGSample_t> buffer;

ADS1256 sensor1(PIN_ADC0_SS,PIN_ADC0_DRDY, ADS1256_15000SPS);

volatile bool bufferFlag=false; // Use as a flag to enable buffer reading from the main loop.
bool wasFull=false; // Use as a flag to determine if there is buffer overflow.


uint8_t currChannel=0; //Data gets collected sequentially for each channel this is the current channel being read

EMGSample_t currSample; // Current sample

static void ISR_NewSample(){
	chSysLockFromISR();

  //Changing from mod to make it more readable

  //Reading is sequential ch0,ch1,ch2,... so on.
  // right now we are reading currChannel

  
  if (currChannel==0){
    currSample.timestamp=((float)(micros() - t0))/1.0e3;
  }
    
  if (sensor1.collectData()) { //True when all channels were collected     
     for (uint8_t i=0;i<4;i++){
      currSample.ch[i]=sensor1.ReadData(i);
     }
     buffer.append(currSample);          
     #if SDCARD
     //SDCard::addEMG(value, EMGTimestampsPtr[chan][i], chan);
     #endif     
  }
	
  //Keep track of the current channel being read
  currChannel=currChannel+1;
  currChannel=(currChannel>=EMG_NUMCHANNELS) ? 0:currChannel;
  chSysUnlockFromISR();
}

  
void start(void){ 
    
    sensor1.setNumChannels(EMG_NUMCHANNELS);    
            
    buffer.init();    
    //Set up ADC chip	
    // Turn the ADS1256 into Continuous read mode
    pinMode(PIN_ADC0_DRDY,INPUT_PULLUP);    
    // Attach the interrupt to the DRDY pin defined by the class
    attachInterrupt(digitalPinToInterrupt(PIN_ADC0_DRDY),ISR_NewSample,FALLING);
    sensor1.startCollecting();	
	}
}
