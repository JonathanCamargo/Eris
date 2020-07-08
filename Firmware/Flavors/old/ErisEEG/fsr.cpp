#include "Eris.h"
#include "fsr.h"

#include <ADC.h>
#include <IntervalTimer.h>

#if SDCARD
#include "sdcard.h"
#endif

namespace FSR{
	
thread_t *generateWave = NULL;
IntervalTimer fsrTimer; 
ADC *adc = new ADC(); // adc object

//Buffer for readings 
ErisBuffer<float> buffer;




static void ISR_fsrTimer(){
  int value=analogRead(PIN_FSR);
  float v=value*(3.3/1024)*(320.0/220.0);
  buffer.append(v);
  #if SDCARD
    SDCard::addFSR(v);
  #endif 
}
	
	void start(void){    
    buffer.init();
    
    adc->setAveraging(4); // set number of averages
    adc->setResolution(10); // set bits of resolution   
    fsrTimer.begin(ISR_fsrTimer,FSR_PERIOD_US);
    
 	}
	
	
	
	
	
	
	
	
	
}
