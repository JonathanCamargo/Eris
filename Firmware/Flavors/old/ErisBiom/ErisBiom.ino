// Eris is a firmware that handles low level control
// It is based on FreeRTOS ported to Arduino (DUE)

#include "configuration.h"
#include "Eris.h"
#include "biom.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "features.h"
#include "serialcommand.h"


 const char firmwareInfo[]=FIRMWARE_INFO;
 char strbuffer[STRBUFFERSIZE]="\0";



void start(){
  sprintf(strbuffer,"%1.2f",12.0);
  
  /*************** Start Threads ************************/    
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)
  
  // start special tasks from extenal sources
  //SineWave::start();
  Features::start();
  Biom::start();
  // Command interfaces   
  SerialCom::start();

  //Register and do an initial default settings for mask
  Features::Default();
 
  /******************************************************/  
 
}

void setup(){  
  Serial.begin(115200);
  //while (!Serial) {
  //  ; // wait for //Serial port to connect. Needed for native USB
  //}
  delay(3000);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  pinMode(PIN_LED_R,OUTPUT); 
  digitalWrite(PIN_LED,LOW); 
  digitalWrite(PIN_LED_R,LOW); 
  analogWrite(3, 10);
  // Setup battery pins
  /******************************************************/
  
  
  /******************************************************/
  //Start threads
  ERIS_RUN(start);
}

void ResetTime() {
  t0 = micros();
}


void loop(){
  eris_sleep_ms(10000);
}
