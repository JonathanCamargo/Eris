#include "Eris.h"
#include "potentiometer.h"
#include "motor.h"

namespace Potentiometer{
	
thread_t *readPotentiometer = NULL;

uint8_t pins[POT_NUMCHANNELS]={PIN_POT_0};//{PIN_POT_0,PIN_POT_1};

//Buffer for readings 
ErisBuffer<PotentiometerSample_t> buffer;

PotentiometerSample_t lastSample;

static THD_WORKING_AREA(waReadPotentiometer_T, 128);
static THD_FUNCTION(ReadPotentiometer_T, arg) {  
  static long idx=0;
  while(1){
    float timestamp = ((float)(micros() - t0))/1.0e3;      
    
    PotentiometerSample_t thisSample;
    
    for (int i=0;i<POT_NUMCHANNELS;i++){
      float value=3.3*analogRead(pins[i])/1024;                    
      thisSample.timestamp=timestamp;
      // Check if it is entering the deadband
      float d=abs(value-lastSample.ch[i]);
      if ((d>0.5)&&(d<2.5)) {
        value=0;
      }
      thisSample.ch[i]=value;                      
    }
    lastSample=thisSample; 
    buffer.append(thisSample); 
    //Update motor state
    Motor::UpdateMeasurement(thisSample.ch[0]);
    chThdSleepMilliseconds(POT_PERIOD_MS);    
  }
}

	
	void start(void){        
    buffer.init();
    // create tasks at priority lowest priority
    readPotentiometer=chThdCreateStatic(waReadPotentiometer_T, sizeof(waReadPotentiometer_T),NORMALPRIO+1, ReadPotentiometer_T, NULL);
	}
	
	
}
