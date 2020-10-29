// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino
// Eris Bici to control the assistance of an electric bike

#include "configuration.h"
#include "Eris.h"
#include "sinewave.h"
#include "serialcommand.h"
#include "btnpwm.h"


long t0=0; // Global start time for all modules 
thread_t *thread1 = NULL;
const char firmwareInfo[]=FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */


/* ************************************************************************************************* */


void start(){
  /*************** Start Threads ************************/    
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  // Command interfaces   
  SerialCom::start();
  ButtonPWM::start();

}

void setup(){  
  Serial.begin(115200);  
  delay(500);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT);      
  //Start threads
  chBegin(start);   
   
  while(true){}
}




void loop(){    
    chThdSleepMilliseconds(10000);
}
