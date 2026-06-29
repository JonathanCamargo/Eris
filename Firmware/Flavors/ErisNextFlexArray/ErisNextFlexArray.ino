// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "emg.h"
#include "fsr.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommand.h"
#include "serialselector.h"

#if SDCARD
#include "sdcard.h"
#endif

#include <SPI.h>

const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  /*************** Start Threads ************************/    
  Heartbeat::start();
  //Error::start(); // Start error notification task (Do not disable)

  #if SDCARD
  SDCard::start();
  #endif
  
  // start special tasks from external sources
  SineWave::start();
  EMG::start();
  SerialSelector::start();
  FSR::start();
  // Command interfaces   
  SerialCom::start();
  
 
  /******************************************************/  
 
}

void setup(){  
  Serial.begin(115200);
  // Wait for USB Serial.
  while (!Serial) {}
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
