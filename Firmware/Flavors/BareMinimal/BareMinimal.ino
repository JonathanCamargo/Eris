// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include <modules/sinewave.h>
#include "serialcommands.h"

#include <SPI.h>

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
  // Initialize mutex for heartbeat
  /*************** Start Threads ************************/    
  Serial.println("start() begin");
  eris_thread_create(waThread1, ERIS_STACK_TINY, ERIS_NORMAL_PRIORITY, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  // Command interfaces   
  SerialCom::start();
  Serial.println("start() done");

}

void setup(){  
  Serial.begin(115200);
  // Wait for USB Serial.
  while (!Serial) {}
  delay(500);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris puto");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  pinMode(PIN_LED_R,OUTPUT); 
  digitalWrite(PIN_LED,LOW); 
  digitalWrite(PIN_LED_R,LOW); 
  analogWrite(3, 10);
  
  //Start threads
  eris_scheduler_start(start);

#ifdef ERIS_USE_FREERTOS
  // On FreeRTOS boards (nRF52), the scheduler is already running.
  // setup() returns and the loop task yields to other threads.
#else
  while(true){}
#endif
}




void loop(){    
    eris_sleep_ms(10000);
}
