// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino
//
// ErisEMGPill: analog EMG Pill front-ends on the built-in ADC.
//   EMG_NUMCHANNELS pills -> ERIS_ADC(0..N-1) @ EMG_HZ   (emgpill.cpp)
// Target: Seeed XIAO ESP32S3 (any Eris board with free ADC pins works).

#include "configuration.h"
#include "Eris.h"
#include "emgpill.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommands.h"

const char firmwareInfo[]=FIRMWARE_INFO;


void start(){
  eriscommon::setPrintPacketMode(true); //send prints to eris

  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  t0=micros();

  // start special tasks from external sources
  SineWave::start();
  EMGPill::begin();   // ADC setup, then the EMGPill sampling thread

  // Serial command interface
  SerialCom::start();

  // Boot streaming as plain text so the Serial Monitor/Plotter shows the EMG
  // with nothing typed. The Python driver sends "S_MODE BIN" on connect.
  SerialCom::bootDefaults("EMG", true);
}

void setup(){
  Serial.begin(ERIS_SERIAL_BAUD);
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
