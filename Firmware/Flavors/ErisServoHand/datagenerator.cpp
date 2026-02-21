#include "Eris.h"

namespace DataGenerator{
	
  //Buffer for readings. See or create data types in customtypes.h
  ErisBuffer<floatSample_t> buffer;

  static const float kFrequency=1.0;

  // DataGenerator thread producing synthetic data for testing purposes
  void Task_DataGenerator(void *pvParameters)  // This is a task.
  {
      (void) pvParameters;    
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
        vTaskDelay(pdMS_TO_TICKS(DATAGENERATOR_PERIOD_MS)); // wait for one second
      
      }
  }
  	
	void start(void){        
    buffer.init();
    
    // Start the DataGenerator task
    xTaskCreate(
    Task_DataGenerator
    ,  "DataGenerator"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
	}

  void PrintSample(floatSample_t sample){
    Console.print(sample.timestamp,2);
    Console.print("(ms), value:");         
    Console.print(sample.value,2);       
  }

}
	
	
	
	
	
	
	
	
