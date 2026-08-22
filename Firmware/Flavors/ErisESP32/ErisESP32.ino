// Eris on the ESP32 family.
//
// The ESP32 Arduino core boots FreeRTOS itself and calls setup() from inside a
// task, so ERIS_RUN() here does NOT start a scheduler and does NOT block — it
// spawns the Eris threads and returns. loop() must therefore yield (it sleeps
// below); a busy loop() would starve the idle task that feeds the watchdog.

#include "configuration.h"
#include "Eris.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommands.h"

const char firmwareInfo[] = FIRMWARE_INFO;

void start() {
  /*************** Start Threads ************************/
  Serial.println("start() begin");
  Heartbeat::start();
  Error::start();   // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  // Command interfaces
  SerialCom::start();
  Serial.println("start() done");
}

void setup() {
  Serial.begin(ERIS_SERIAL_BAUD);
  // Classic ESP32 talks over a UART bridge, where `while(!Serial)` is always
  // true immediately. On native-USB parts (S2/S3/C3) it waits for the host, so
  // give it a bounded wait rather than blocking an untethered board forever.
  unsigned long t = millis();
  while (!Serial && (millis() - t) < 2000) {}
  delay(500);

  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Start threads. Returns immediately on ESP32 — see the note at the top.
  ERIS_RUN(start);
}

void loop() {
  // Must yield: the Eris threads share this core, and the idle task needs to
  // run for the task watchdog.
  eris_sleep_ms(10000);
}
