// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "motor.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "potentiometer.h"
#include "serialcommand.h"

#include <SPI.h>

const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  // Initialize mutex for heartbeat
  /*************** Start Threads ************************/    
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  Motor::start();
  SineWave::start();
  Potentiometer::start();
  // Command interfaces   
  SerialCom::start();

}

void setup(){  
  Serial.begin(115200);
  // Wait for USB Serial.
  while (!Serial) {}
  delay(500);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
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
