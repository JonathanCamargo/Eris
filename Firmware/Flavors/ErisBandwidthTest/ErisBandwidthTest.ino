// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "multiwave.h"
#include "serialcommand.h"

#include <SPI.h>


const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)
  // start special tasks from external sources
  SineWave::start();
  MultiWave::start();
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

  //Start threads
  ERIS_RUN(start);
}




void loop(){
    eris_sleep_ms(10000);
}
