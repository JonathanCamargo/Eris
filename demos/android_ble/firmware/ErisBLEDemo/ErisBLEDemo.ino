// Eris BLE demo firmware -- the device side of demos/android_ble.
//
// Streams a few no-wiring signals over Bluetooth LE (Nordic UART Service) and
// accepts commands from the Android app. Target: Seeed XIAO nRF52840.

#include "configuration.h"
// A literal include of <bluefruit.h> is REQUIRED for BLE: Arduino only adds a
// library's include path when a sketch source names one of its headers, and
// the eriscommon BLE module compiles to nothing without it.
#ifdef ERIS_USE_BLE
#include <bluefruit.h>
#endif
#include "Eris.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include <modules/ble_transport.h>
#include "serialcommands.h"
#include "signals.h"

const char firmwareInfo[] = FIRMWARE_INFO;

void start() {
  // Route eriscommon prints (error reports included) into packets, so they
  // reach the phone instead of the unplugged USB port.
  eriscommon::setPrintPacketMode(true);

  Heartbeat::start();
  Error::start();

  t0 = micros();

  SineWave::start();
  Waves::start();
  Analog::start();
  Temp::start();

  // Commands + streaming. Nothing streams until the app sends S_F and S_ON.
  SerialCom::start();
}

void setup() {
  Serial.begin(115200);
  // No while(!Serial): an untethered board has no USB host and would hang.
  delay(500);

#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, RGB_LED_OFF);
  digitalWrite(LED_GREEN, RGB_LED_OFF);
  digitalWrite(LED_BLUE, RGB_LED_OFF);
#endif
  analogReadResolution(12);

#ifdef ERIS_USE_BLE
  SerialCom::useBLE(ERIS_BLE_NAME);   // before ERIS_RUN: start() does the swap
#endif

  ERIS_RUN(start);
}

void loop() {
  eris_sleep_ms(10000);
}
