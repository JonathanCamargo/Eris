// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino
//
// ErisFSRCalibrator: press on an FSR with a stepper and read the true force
// under it with a load cell.
//   A4988  STEP/DIR/EN -> motion   (motion.cpp, G0 via gcode.cpp)
//   HX711  DOUT/SCK    -> force    (loadcell.cpp)
//   FSR    divider     -> voltage  (loadcell.cpp, same tick as the force)

#include "configuration.h"
#include "Eris.h"
#include "motion.h"
#include "loadcell.h"
#include "gcode.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "serialcommands.h"

// Literal include so the Arduino builder finds and links the HX711 library
// (it only discovers libraries from #include lines it can see in the sketch
// and the sketch's own sources -- see lessons_learned / library discovery).
#include <HX711.h>

const char firmwareInfo[]=FIRMWARE_INFO;


void start(){
  eriscommon::setPrintPacketMode(true); //send prints to eris

  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  t0=micros();

  // start special tasks from external sources
  SineWave::start();

  // Motion before the load cell: the overload guard drives the axis.
  Motion::begin();
  Loadcell::begin();   // also starts the FSRCal sampling thread

  // Serial command interface
  SerialCom::start();

  // Boot streaming as plain text so the Serial Monitor/Plotter shows voltage,
  // force and position with nothing typed. The Python driver sends
  // "S_MODE BIN" on connect.
  SerialCom::bootDefaults("FSR", true);
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
