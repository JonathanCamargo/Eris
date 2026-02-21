#include "Eris.h"

namespace DataGenerator{
	
  //Buffer for readings. See or create data types in customtypes.h
  ErisBuffer<floatSample_t> buffer;

  static const float kFrequency=1.0;

  // DataGenerator thread producing synthetic data for testing purposes
  ERIS_THREAD_FUNC(Task_DataGenerator) {
      long idx=0;
      while(true){
        if (idx>=10000){
        idx=0;
        }
        else{
        idx=idx+1;
        }

        float timestamp = ((float)(micros() - Eris::t0))/1.0e3;
        float value=3.3*sin(2*M_PI*kFrequency*((float)idx)/1.0e4);
        floatSample_t thisSample;

        thisSample.timestamp=timestamp;
        thisSample.value=value;
        buffer.append(thisSample);
        eris_sleep_ms(DATAGENERATOR_PERIOD_MS); // wait for one second

      }
  }
  	
	void start(void){        
    buffer.init();
    
    // Start the DataGenerator task
    ERIS_THREAD_WA(waDataGenerator, 128*sizeof(StackType_t));
    eris_thread_create(waDataGenerator, 128*sizeof(StackType_t), 1, Task_DataGenerator, NULL);
	}

  void PrintSample(floatSample_t sample){
    Console.print(sample.timestamp,2);
    Console.print("(ms), value:");         
    Console.print(sample.value,2);       
  }

}
	
	
	
	
	
	
	
	
