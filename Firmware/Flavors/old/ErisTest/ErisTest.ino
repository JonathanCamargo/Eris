// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"

#include "emg.h"
#include "fsr.h"
#include "sinewave.h"
#include "serialcommand.h"

#if SDCARD
#include "sdcard.h"
#endif

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

  #if SDCARD
  SDCard::start();
  #endif
  
  // start special tasks from extenal sources
  SineWave::start();
  EMG::start();
  FSR::start();
  // Serial Command interfaces   
  SerialCom::start();
  
 
  /******************************************************/  
 
}

void setup(){  
  Serial.begin(115200); 
  delay(1000);
  // Setup the initial configuration  
  eriscommon::printText("HELLO, This is Eris"); 
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  digitalWrite(PIN_LED,LOW); 
   
  pinMode(A3,OUTPUT); 
  tone(A3,1000);
  /******************************************************/  
  
  /******************************************************/
  //Start threads
  chBegin(start);   
  while(true){}

}




void loop(){  
  //This is not used since FreeRTOS takes charge
}
