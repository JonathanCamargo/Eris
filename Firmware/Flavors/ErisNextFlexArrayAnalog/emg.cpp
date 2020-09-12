#include "Eris.h"
#include "emg.h"
#include "serialcommand.h"
#include "configuration.h"

#include <IntervalTimer.h>

namespace EMG{

IntervalTimer timer0; // Timer for ADC
  
thread_t *extractFeaturesEMG = NULL;
thread_t *readSensor1 = NULL;

//Buffer for readings 
ErisBuffer<EMGSample_t> buffer;

volatile bool bufferFlag=false; // Use as a flag to enable buffer reading from the main loop.
bool wasFull=false; // Use as a flag to determine if there is buffer overflow.


uint8_t currChannel=0; //Data gets collected sequentially for each channel this is the current channel being read

static void ISR_NewSample(){
	chSysLockFromISR();

  EMGSample_t currSample; // Current sample
  float timestamp = ((float)(micros() - t0))/1.0e3;     
  currSample.timestamp=timestamp;
  for (uint8_t i=0;i<EMG_NUMCHANNELS;i++){
    int a=analogRead(PINS_EMG[i]);  
   currSample.ch[i]=(float)(a)*3.3/1024;
  buffer.append(currSample); 
  }         
  chSysUnlockFromISR();
}

  
void start(void){                  
    buffer.init();                
    timer0.begin(ISR_NewSample, EMG_PERIOD_US);   
	}
 
}
