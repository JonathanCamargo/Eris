// ErisServo is an Eris firmware flavor for PCA9685 servo control
// Drives up to 16 servos via I2C using Adafruit PWM Servo Driver
// Based on ErisMinimal's portable foundation (no Teensy-specific code)

#include "configuration.h"
#include "Eris.h"
#include "servos.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "servo_commands.h"

const char firmwareInfo[]=FIRMWARE_INFO;


void start(){
  /*************** Start Threads ************************/
  Serial.println("Creating threads...");
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  Servos::start();
  // Command interfaces
  SerialCom::start();
  Serial.println("start() done");

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
