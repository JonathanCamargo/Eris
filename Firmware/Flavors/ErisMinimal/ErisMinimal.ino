// ErisMinimal is a stripped-down Eris firmware flavor
// for generic Arduino boards (no Teensy-specific code, no SD card)
// It is based on RTOS and ported to Arduino

#include "configuration.h"
#include "Eris.h"
#include "fsr.h"
#include "sync.h"
#include "sinewave.h"
#include "serialcommand.h"


long t0=0; // Global start time for all modules

eris_thread_ref_t thread1 = NULL;


const char firmwareInfo[]=FIRMWARE_INFO;

/* ******************************** Global threads ************************************************** */

// Heartbeat thread
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
  /*************** Start Threads ************************/
  thread1 = eris_thread_create(waThread1, ERIS_STACK_TINY, ERIS_NORMAL_PRIORITY, Thread1, NULL);
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  FSR::start();
  Sync::start();
  // Command interfaces
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

// FreeRTOS static allocation callbacks (required when configSUPPORT_STATIC_ALLOCATION is enabled)
#ifdef ERIS_USE_FREERTOS
extern "C" {

static StaticTask_t xIdleTaskTCB;
static StackType_t  uxIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB;
static StackType_t  uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

}
#endif
