// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "Eris.h"
#include <modules/heartbeat.h>

#include <SPI.h>



const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  /*************** Start Threads ************************/    
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  ADS131::start();
  // Command interfaces   
  SerialCom::start();
  
 
  /******************************************************/  
 
}

void setup(){  
  Serial.begin(115200);
  delay(1000);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  digitalWrite(PIN_LED,HIGH); 
  // Setup battery pins
  // SPI chip select
  /******************************************************/
  SPI.begin();
  /******************************************************/
  //Start threads
  ERIS_RUN(start);
}

void loop(){  
    eris_sleep_ms(10000);
}
