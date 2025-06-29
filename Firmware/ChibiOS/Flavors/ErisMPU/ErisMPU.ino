// Eris is a firmware that handles low level control
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"

#include "fsr.h"
#include "imu.h"
#include "sinewave.h"
#include "serialcommand.h"

#include <SPI.h>

thread_t *thread1 = NULL;

long t0=0; // Global time

const char firmwareInfo[]=FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */

// Mutex to enable or disable heartbeat
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  while (1) {
    // Sleep for 1000 milliseconds.
    // Toggle pin to show heartbeat    
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    chThdSleepMilliseconds(250);
  }
}
/* ************************************************************************************************* */


void start(){
  eriscommon::setPrintPacketMode(true); //send prints to eris
  
  // Initialize mutex for heartbeat
  //chMtxObjectInit(&mtxhb);
  /*************** Start Threads ************************/    
  chThdCreateStatic(waThread1, sizeof(waThread1),
                                   NORMALPRIO, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)

  t0=micros();

  // start special tasks from external sources
  SineWave::start();
  IMU::start();
  FSR::start();
  
  // Serial command interface   
  SerialCom::start();

  

}

void setup(){  
  Serial.begin(115200);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,LOW);
  // Wait for USB Serial.  
  delay(2000);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  pinMode(PIN_LED_R,OUTPUT); 
  digitalWrite(PIN_LED,LOW); 
  digitalWrite(PIN_LED_R,LOW); 
  // Setup battery pins
    
  /******************************************************/

  /******************************************************/
  //Start threads
  chBegin(start);   
  digitalWrite(PIN_LED,HIGH); 
  while(true){}
}




void loop(){    
    chThdSleepMilliseconds(10000);
}
