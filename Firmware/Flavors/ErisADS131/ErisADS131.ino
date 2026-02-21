// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "Eris.h"

#include <SPI.h>

long t0=0; // Global start time for all modules 

eris_thread_ref_t thread1 = NULL;


const char firmwareInfo[]=FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */
ERIS_THREAD_WA(waThread1, 32);
ERIS_THREAD_FUNC(Thread1) {
  while (1) {
    // Sleep for 1000 milliseconds.
    eris_sleep_ms(1000);
    // Toggle pin to show heartbeat    
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));           
  }
}
/* ************************************************************************************************* */


void start(){
  /*************** Start Threads ************************/    
  eris_thread_create(waThread1, 32, NORMALPRIO, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  ADS131::start();
  // Command interfaces   
  SerialCom::start();
  
 
  /******************************************************/  
 
}

void setup(){  
  Serial.begin(115200);
  delay(1000);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  digitalWrite(PIN_LED,HIGH); 
  // Setup battery pins
  // SPI chip select
  /******************************************************/
  SPI.begin();
  /******************************************************/
  //Start threads
  eris_scheduler_start(start);
  while(true){}
}

void loop(){  
    eris_sleep_ms(10000);
}
