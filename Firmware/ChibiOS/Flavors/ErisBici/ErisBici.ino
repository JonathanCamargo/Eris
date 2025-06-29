// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino
// Eris Bici to control the assistance of an electric bike

#include "configuration.h"
#include "Eris.h"
#include "btnpwm.h"


//long t0=0; // Global start time for all modules 
const char firmwareInfo[]=FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */

/* ************************************************************************************************* */


void start(){
  /*************** Start Threads ************************/    
  //Error::start(); // Start error notification task (Do not disable)
  // start special tasks from external sources
  // Command interfaces     
  ButtonPWM::start();

}

void setup(){  
  pinMode(PIN_LED,OUTPUT);  
  digitalWrite(PIN_LED,LOW);
  Serial.begin(115200);  
  delay(500);  
  // Setup the initial configuration  
  Serial.println(F("HELLO, This is Eris"));
  /*************** Configure HW pins *******************/    
  //Start threads
  chBegin(start);      
  while(true){}
}



void loop(){    
    chThdSleepMilliseconds(10000);
}
