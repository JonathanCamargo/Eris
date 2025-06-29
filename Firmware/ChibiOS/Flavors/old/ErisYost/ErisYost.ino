// Eris is a firmware that handles low level control
// It is based on FreeRTOS ported to Arduino (DUE)

#include "configuration.h"
#include "Eris.h"
#include "emg.h"
#include "imu.h"
#include "sinewave.h"
#include "serialcommand.h"



#if SDCARD
#include "sdcard.h"
#endif

#include <SPI.h>

thread_t *thread1 = NULL;


 const char firmwareInfo[]=FIRMWARE_INFO;
 char strbuffer[STRBUFFERSIZE]="\0";

/* ******************************** Global threads ************************************************** */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  // A thread for heart beat the LED
  while (1) {
    // Sleep for 1000 milliseconds.
    chThdSleepMilliseconds(1000);
    // Toggle pin to show heartbeat    
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));           
  }
}
/* ************************************************************************************************* */


void start(){
  sprintf(strbuffer,"%1.2f",12.0);
  /*************** Start Threads ************************/    
  chThdCreateStatic(waThread1, sizeof(waThread1),
                                   NORMALPRIO-10, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)

  #if SDCARD
  //SDCard::start();
  #endif
  
  // start special tasks from extenal sources
  //SineWave::start();
  //EMG::start();
  IMU::start();
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
  pinMode(PIN_LED,OUTPUT); 
  digitalWrite(PIN_LED,HIGH); 
  pinMode(PIN_ERROR,OUTPUT);
  pinMode(PIN_SS0,OUTPUT); 
  digitalWrite(PIN_SS0,HIGH); 
  // Setup battery pins
  /******************************************************/
  
  SPI.begin();

  /******************************************************/
  //Start threads
  chBegin(start);   
  while(true){}
}




void loop(){  
  //This is not used since FreeRTOS takes charge
}
