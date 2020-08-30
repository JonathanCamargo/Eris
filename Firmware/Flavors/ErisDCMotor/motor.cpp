#include "Eris.h"
#include "motor.h"
#include "PID_v1.h"

namespace Motor{
	
thread_t *updatepid = NULL;

//Buffer for readings 
double input=0;
double output=0;
double setpoint=2.0;

PID pid(&input,&output,&setpoint,10.0,0.1,0.1,1,1);

static THD_WORKING_AREA(waUpdatePID_T, 128);
static THD_FUNCTION(UpdatePID_T, arg) {  
  static long idx=0;
  while(1){        
    if (pid.Compute()){
      Serial.print(input); 
      Serial.print(",");
      Serial.println(output);
    }         
    chThdSleepMilliseconds(20);    
  }
}

	
	void start(void){        
    //buffer.init();
    // create tasks at priority lowest priority
    updatepid=chThdCreateStatic(waUpdatePID_T, sizeof(waUpdatePID_T),NORMALPRIO+1, UpdatePID_T, NULL);
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
