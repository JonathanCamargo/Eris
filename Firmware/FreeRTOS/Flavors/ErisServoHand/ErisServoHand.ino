 ///////////////////////////////////////////////////////////////////////////////////
// Project: ErisServoHand
// Description: Use PCA9685 I2C Servo driver to move a robotic hand based on serial commands
//              Commands are defined in serialcommand.cpp
//              - OPEN
//              - CLOSE
//              - SHAKA
//              - X <#0-100 APERTURE FOR DOF 1> <DOF2> ... <DOF5> 
// 
// About Eris: Eris is a firmware enables complex operations on uC using RTOS ported to Arduino.
// Includes:
// - Efficient data buffering and streaming
// - A robust RT framework based on ChibiOS or FreeRTOS

#include "Eris.h"
#include "configuration.h"



/* ******************************** Global tasks ************************************************** */
// Heartbeat thread blinking PIN_LED to show that the system is alive
void Task_Heartbeat(void *pvParameters)  // This is a task.
{
    (void) pvParameters;    
    // initialize digital LED_BUILTIN on pin 13 as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    for (;;) // A Task shall never return or exit.
    {
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)); // wait for one second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)); // wait for one second
    }
}
/* ************************************************************************************************* */


void start(){

  /*************** Start Threads ************************/    

  // Start heartbeat task
  xTaskCreate(
    Task_Heartbeat
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  //Error::start(); // Start error notification task (Do not disable)
  
  ///////////////////////
  /// Start tasks
  DataGenerator::start();
  SerialCom::start();
  Servos::start();
  vTaskStartScheduler();
}

void setup(){  
  CONSOLE_PORT.begin(115200);
  STREAM_PORT.begin(115200);
  
  // Wait for Serial.
  while (!CONSOLE_PORT) {}
  while (!STREAM_PORT) {}    
  delay(3000);

  // Setup the initial configuration  
  Console.println("HELLO, This is Eris");
  
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT);   
  digitalWrite(PIN_LED,LOW);   
  
  // Start tasks
  start();

  // Keep inside this loop 
  while(true){}
}

void loop(){        
     // Empty. Everything is done in Tasks.
}
