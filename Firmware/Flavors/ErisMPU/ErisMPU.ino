// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "imu.h"
#include "sinewave.h"
#include "serialcommand.h"

#include <Wire.h>

long t0=0; // Global time

eris_thread_ref_t thread1 = NULL;

const char firmwareInfo[]=FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */

// Mutex to enable or disable heartbeat
ERIS_THREAD_WA(waThread1, ERIS_STACK_TINY);
ERIS_THREAD_FUNC(Thread1) {
  while (1) {
    // Sleep for 1000 milliseconds.
    // Toggle pin to show heartbeat    
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    eris_sleep_ms(250);
  }
}
/* ************************************************************************************************* */


void start(){
  eriscommon::setPrintPacketMode(true); //send prints to eris
  
  // Initialize mutex for heartbeat
  //chMtxObjectInit(&mtxhb);
  /*************** Start Threads ************************/    
  thread1 = eris_thread_create(waThread1, ERIS_STACK_TINY, ERIS_NORMAL_PRIORITY, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)

  t0=micros();

  // start special tasks from external sources
  SineWave::start();
  IMU::start();  
  
  // Serial command interface   
  SerialCom::start();

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
  eris_scheduler_start(start);  
  while(true){}
}




void loop(){    
    eris_sleep_ms(10000);
}
