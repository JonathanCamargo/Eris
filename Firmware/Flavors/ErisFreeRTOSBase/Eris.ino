// Eris is a firmware enables complex operations on uC using RTOS ported to Arduino.
// Includes:
// - Efficient data buffering and streaming
//
#include "Eris.h"
#include "configuration.h"

/* ******************************** Global tasks ************************************************** */

// Heartbeat thread blinking PIN_LED to show that the system is alive
ERIS_THREAD_FUNC(Task_Heartbeat) {
    // initialize digital LED_BUILTIN on pin 13 as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    for (;;) // A Task shall never return or exit.
    {
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        eris_sleep_ms(HEARTBEAT_PERIOD_MS); // wait for one second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        eris_sleep_ms(HEARTBEAT_PERIOD_MS); // wait for one second
    }
}
/* ************************************************************************************************* */


void start(){

  /*************** Start Threads ************************/    

  // Start heartbeat task
  ERIS_THREAD_WA(waHeartbeat, 128*sizeof(StackType_t));
  eris_thread_create(waHeartbeat, 128*sizeof(StackType_t), 2, Task_Heartbeat, NULL);

  //Error::start(); // Start error notification task (Do not disable)
  
  ///////////////////////
  /// Start tasks
  DataGenerator::start();
  SerialCom::start();
}

void setup(){  
  CONSOLE_PORT.begin(115200);
  STREAM_PORT.begin(115200);
  
  // Wait for Serial.
  while (!CONSOLE_PORT) {}
  while (!CONSOLE_PORT) {}    
  delay(3000);

  // Setup the initial configuration  
  Console.println("HELLO, This is Eris");
  
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT);   
  digitalWrite(PIN_LED,LOW);   
  
  // Start tasks
  eris_scheduler_start(start);

  // Keep inside this loop
  while(true){}
}

void loop(){        
     // Empty. Everything is done in Tasks.
}
