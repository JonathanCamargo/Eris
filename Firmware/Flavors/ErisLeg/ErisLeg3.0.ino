// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "fsr.h"
#include "sync.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommand.h"
#include "joints.h"
#include "loadcell.h"

#include <SPI.h>


const char firmwareInfo[] = FIRMWARE_INFO;



void start() {
  // Initialize mutex for heartbeat
  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)
  eriscommon::setPrintPacketMode(true);
  // start special tasks from external sources
  SineWave::start();
  FSR::start();
  Sync::start();
  Joints::start();
  Loadcell::start();
  // Command interfaces
  SerialCom::start();

  t0 = micros();

}

void setup() {
  Serial.begin(115200);
  // Wait for USB Serial.
  while (!Serial) {}
  delay(1000);
  // Setup the initial configuration
  eriscommon::println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LED_R, LOW);
  analogWrite(3, 10);
  // Setup battery pins
  // SPI chip select
  /******************************************************/
  SPI.begin();
  /******************************************************/
  //Start threads
  ERIS_RUN(start);

  //Test analog write


}




void loop() {
  eris_sleep_ms(10000);
}
