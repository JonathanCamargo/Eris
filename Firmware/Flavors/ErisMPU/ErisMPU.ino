// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "imu.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommands.h"

#include <Wire.h>

const char firmwareInfo[]=FIRMWARE_INFO;


void start(){
  eriscommon::setPrintPacketMode(true); //send prints to eris

  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  t0=micros();

  // start special tasks from external sources
  SineWave::start();
  IMU::begin();  
  
  // Serial command interface   
  SerialCom::start();

  // Boot streaming both IMUs as plain text, so plugging the board in and
  // opening the Serial Monitor (or Plotter) shows data with nothing typed.
  // The Python driver sends "S_MODE BIN" on connect to switch to the binary
  // protocol. Pass false here to boot straight into binary instead.
  SerialCom::bootDefaults("IMU_0 IMU_1", true);

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
  digitalWrite(PIN_LED,LOW);   
  /******************************************************/

  //Start threads
  ERIS_RUN(start);
}




void loop(){    
    eris_sleep_ms(10000);
}
