// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "fsr.h"
#include "sync.h"
#include "sinewave.h"
#include "serialcommand.h"
#include "joints.h"
#include "loadcell.h"

#include <SPI.h>

long t0 = 0;

eris_thread_ref_t thread1 = NULL;

const char firmwareInfo[] = FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */

// Mutex to enable or disable heartbeat
ERIS_THREAD_WA(waThread1, 32);
ERIS_THREAD_FUNC(Thread1) {
  while (1) {
    // Sleep for 1000 milliseconds.
    // Toggle pin to show heartbeat
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    eris_sleep_ms(250);
  }
}
/* ************************************************************************************************* */


void start() {
  // Initialize mutex for heartbeat
  //chMtxObjectInit(&mtxhb);
  /*************** Start Threads ************************/
  eris_thread_create(waThread1, 32,
                    NORMALPRIO, Thread1, NULL);
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
  eris_scheduler_start(start);

  //Test analog write


  while (true) {}
}




void loop() {
  eris_sleep_ms(10000);
}
