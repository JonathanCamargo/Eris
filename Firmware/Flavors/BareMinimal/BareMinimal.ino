// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommands.h"

#include <SPI.h>

const char firmwareInfo[]=FIRMWARE_INFO;


void start(){
  /*************** Start Threads ************************/
  Serial.println("start() begin");
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  // Command interfaces   
  SerialCom::start();
  Serial.println("start() done");

}

void setup(){  
  Serial.begin(115200);
  // Wait for USB Serial.
  while (!Serial) {}
  delay(500);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris puto");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  pinMode(PIN_LED_R,OUTPUT); 
  digitalWrite(PIN_LED,LOW); 
  digitalWrite(PIN_LED_R,LOW); 
  analogWrite(3, 10);
  
  //Start threads
  ERIS_RUN(start);
}




void loop(){    
    eris_sleep_ms(10000);
}
