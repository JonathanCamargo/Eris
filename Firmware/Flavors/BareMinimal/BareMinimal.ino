// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
// A literal include of <bluefruit.h> here is REQUIRED when BLE is enabled, even
// though we never call Bluefruit directly from the sketch. Arduino only adds a
// library's include path to the build if some source #includes one of its
// headers; the BLE module in eriscommon is guarded on __has_include(<bluefruit.h>),
// which would otherwise always be false (the path is never added) and silently
// compile BLE out. This one line pulls Bluefruit onto the path for every
// translation unit, so the guard flips true and the BLE code compiles.
#ifdef ERIS_USE_BLE
#include <bluefruit.h>
#endif
#include "Eris.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include <modules/ble_transport.h>
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
  // Wait for USB Serial. Skip when BLE is the transport: an untethered device
  // has no USB host, so this would hang forever.
#ifndef ERIS_USE_BLE
  while (!Serial) {}
#endif
  delay(500);
  // Select the BLE transport before threads spawn. This call lives in the
  // sketch (not the eriscommon library), so the configuration.h flag reaches
  // it; SerialCom::start() then repoints the data + command paths onto BLE.
#ifdef ERIS_USE_BLE
  SerialCom::useBLE(ERIS_BLE_NAME);
#endif
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
