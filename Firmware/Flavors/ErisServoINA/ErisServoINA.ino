// ErisServoINA is an Eris firmware flavor for PCA9685 servo control
// with INA219 current sensing. Streams current/voltage/power at 100Hz.
// Drives up to 16 servos via I2C using Adafruit PWM Servo Driver

#include "configuration.h"
#include "Eris.h"
#include "servos.h"
#include "ina219.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommand.h"



const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  Servos::start();
  INA219::start();
  // Command interfaces
  SerialCom::start();

}

void setup(){
  Serial.begin(115200);
  // Wait for USB Serial.
  while (!Serial) {}
  delay(1000);
  // Setup the initial configuration
  Serial.println("HELLO, This is ErisServoINA");
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

