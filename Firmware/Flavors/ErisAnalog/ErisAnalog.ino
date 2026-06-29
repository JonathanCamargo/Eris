// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino
// Flavor: ErisNextFlexAnalog: to read analog and TI information straight from the amplifier cable

#include "configuration.h"
#include "Eris.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "analog.h"
#include "serialcommand.h"

Analog::Driver<ANALOG_NUMCHANNELS> analog;
const uint32_t analog_pins[ANALOG_NUMCHANNELS] = {PINS_ANALOG};



 const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  /*************** Start Threads ************************/    
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  analog.start(analog_pins, ANALOG_PERIOD_US);
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
  /******************************************************/  
  /******************************************************/
  //Start threads
  ERIS_RUN(start);
}




void loop(){  
      eris_sleep_ms(10000);
}
