// ErisServo is an Eris firmware flavor for PCA9685 servo control
// Drives up to 16 servos via I2C using Adafruit PWM Servo Driver
// Based on ErisMinimal's portable foundation (no Teensy-specific code)

#include "configuration.h"
#include "Eris.h"
#include "servos.h"
#include <modules/sinewave.h>
#include "servo_commands.h"

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
  Serial.println("Creating threads...");
  thread1 = eris_thread_create(waThread1, ERIS_STACK_TINY, ERIS_NORMAL_PRIORITY, Thread1, NULL);
  Serial.print("thread1="); Serial.println((uintptr_t)thread1, HEX);
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  Servos::start();
  // Command interfaces
  SerialCom::start();
  Serial.println("start() done");

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

#ifdef ERIS_USE_FREERTOS
  // On FreeRTOS boards (nRF52), the scheduler is already running.
  // setup() returns and the loop task yields to other threads.
#else
  // On ChibiOS (Teensy/SAMD), chBegin() starts the scheduler and
  // setup() never returns.
  while(true){}
#endif
}




void loop(){
    eris_sleep_ms(10000);
}

// FreeRTOS static allocation callbacks (required when configSUPPORT_STATIC_ALLOCATION is enabled).
// On nRF52, the core's rtos.cpp already provides these, so skip them there.
#if defined(ERIS_USE_FREERTOS) && !defined(NRF52_SERIES)
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
