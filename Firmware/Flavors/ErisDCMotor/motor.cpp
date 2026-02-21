#include "Eris.h"
#include "motor.h"
#include "PID_v1.h"

namespace Motor{
	
eris_thread_ref_t updatepid = NULL;

//Buffer for readings 
double input=0;
double output=0;
double setpoint=2.0;

PID pid(&input,&output,&setpoint,10.0,0.1,0.1,1,1);

ERIS_THREAD_WA(waUpdatePID_T, 128);
ERIS_THREAD_FUNC(UpdatePID_T) {  
  static long idx=0;
  while(1){        
    if (pid.Compute()){
      Serial.print(input); 
      Serial.print(",");
      Serial.println(output);
    }         
    eris_sleep_ms(20);    
  }
}

	
	void start(void){        
    //buffer.init();
    // create tasks at priority lowest priority
    updatepid=eris_thread_create(waUpdatePID_T, 128,NORMALPRIO+1, UpdatePID_T, NULL);
    pinMode(PIN_MOT_0_A,OUTPUT);
    pinMode(PIN_MOT_0_B,OUTPUT);
    pid.SetMode(AUTOMATIC);
	}

 void UpdateMeasurement(double angle){
  input=angle;  
 }



  void Forward(){
    digitalWrite(PIN_MOT_0_A,HIGH);
    digitalWrite(PIN_MOT_0_B,LOW);    
  }
  
  void Backward(){
    digitalWrite(PIN_MOT_0_A,LOW);
    digitalWrite(PIN_MOT_0_B,HIGH);
  }
  
  void Idle(){
    digitalWrite(PIN_MOT_0_A,LOW);
    digitalWrite(PIN_MOT_0_B,LOW);    
  }
  
  void Break(){
    digitalWrite(PIN_MOT_0_A,HIGH);
    digitalWrite(PIN_MOT_0_B,HIGH);    
  }

  
	
}
