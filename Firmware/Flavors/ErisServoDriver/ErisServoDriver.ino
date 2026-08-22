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
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  Servos::start();
  // Command interfaces
  SerialCom::start();

}

void setup(){
  Serial.begin(ERIS_SERIAL_BAUD);
  // Wait for the USB host, but bounded: on a board whose Serial is a plain UART
  // (ESP32 via CP2102/CH340) `!Serial` never clears, so an unbounded wait would
  // hang a board that has no host attached.
  unsigned long t0_wait = millis();
  while (!Serial && (millis() - t0_wait) < 2000) {}
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

