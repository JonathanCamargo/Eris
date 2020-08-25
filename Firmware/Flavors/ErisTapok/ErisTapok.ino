// Eris is a firmware that handles low level control
// It is based on FreeRTOS ported to Arduino (DUE)

#include "configuration.h"
#include "Eris.h"

#include "emg.h"
#include "fsr.h"
#include "sync.h"
#include "sinewave.h"
#include "serialcommand.h"

long t0=0; // Global start time for all modules 
thread_t *thread1 = NULL;
 const char firmwareInfo[]=FIRMWARE_INFO;
  
/* ******************************** Global threads ************************************************** */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  while (1) {
    // Sleep for 1000 milliseconds.
    chThdSleepMilliseconds(1000);
    // Toggle pin to show heartbeat    
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));           
  }
}
/* ************************************************************************************************* */

void start(){  
  /*************** Start Threads ************************/    
  chThdCreateStatic(waThread1, sizeof(waThread1),
                                   NORMALPRIO, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)
  
  // start special tasks from extenal sources
  SineWave::start();
  EMG::start();
  FSR::start();
  Sync::start();
  // Command interfaces   
  SerialCom::start();  
  /******************************************************/   
}

void setup(){  
  // Initialize mutex for heartbeat
  //chMtxObjectInit(&mtxhb);
  Serial.begin(115200);
  //while (!Serial) {
  //  ; // wait for //Serial port to connect. Needed for native USB
  //}
  delay(1500);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  digitalWrite(PIN_LED,HIGH); 
  pinMode(PIN_ERROR,OUTPUT);
  // Setup battery pins
  /******************************************************/
  
  
  /******************************************************/
  //Start threads
  chBegin(start);   
  while(true){}
}




void loop(){  
  chThdSleepMilliseconds(10000);
}
