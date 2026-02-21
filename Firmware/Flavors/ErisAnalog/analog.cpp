#include "Eris.h"
#include "analog.h"
#include "serialcommand.h"
#include "configuration.h"

#include <IntervalTimer.h>

namespace Analog{

uint32_t pins[ANALOG_NUMCHANNELS]={PINS_ANALOG};

IntervalTimer timer0; // Timer for ADC
  
//thread_t *extractFeaturesEMG = NULL;
thread_t *readAnalog = NULL;

//Buffer for readings 
ErisBuffer<AnalogSample_t> buffer;

static void ISR_NewSample(){
	chSysLockFromISR();

  AnalogSample_t currSample; // Current sample
  float timestamp = ((float)(micros() - t0))/1.0e3;     
  currSample.timestamp=timestamp;
  int a=0;
  for (uint8_t i=0;i<ANALOG_NUMCHANNELS;i++){
    a=analogRead(pins[i]);  
    currSample.ch[i]=(float)(a)*3.3/1024;
  }
  buffer.append(currSample);            
  chSysUnlockFromISR();
}

  
void start(void){                  
    buffer.init();                
    timer0.begin(ISR_NewSample, ANALOG_PERIOD_US);   
	}
 
}
