// Eris is a firmware that handles low level control
// It is based on FreeRTOS ported to Arduino (DUE)

#include "configuration.h"
#include "Eris.h"
#include "biom.h"
#include "sinewave.h"
#include "features.h"
#include "serialcommand.h"

eris_thread_ref_t thread1 = NULL;

//Global start time for all nodes
long t0 = 0;

 const char firmwareInfo[]=FIRMWARE_INFO;
 char strbuffer[STRBUFFERSIZE]="\0";

/* ******************************** Global threads ************************************************** */
ERIS_THREAD_WA(waThread1, 32);
ERIS_THREAD_FUNC(Thread1) {
  // A thread for heart beat the LED
  while (1) {
    // Sleep for 1000 milliseconds.
    eris_sleep_ms(1000);
    // Toggle pin to show heartbeat
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));
  }
}
/* ************************************************************************************************* */


void start(){
  sprintf(strbuffer,"%1.2f",12.0);
  
  /*************** Start Threads ************************/    
  eris_thread_create(waThread1, 32,
                                   NORMALPRIO-10, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)
  
  // start special tasks from extenal sources
  //SineWave::start();
  Features::start();
  Biom::start();
  // Command interfaces   
  SerialCom::start();
  
 
  /******************************************************/  
 
}

void setup(){  
  Serial.begin(115200);
  //while (!Serial) {
  //  ; // wait for //Serial port to connect. Needed for native USB
  //}
  delay(3000);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  pinMode(PIN_LED_R,OUTPUT); 
  digitalWrite(PIN_LED,LOW); 
  digitalWrite(PIN_LED_R,LOW); 
  analogWrite(3, 10);
  // Setup battery pins
  /******************************************************/
  
  
  /******************************************************/
  //Start threads
  eris_scheduler_start(start);
  while(true){}
}

void ResetTime() {
  t0 = micros();
}


void loop(){
  eris_sleep_ms(10000);
}
