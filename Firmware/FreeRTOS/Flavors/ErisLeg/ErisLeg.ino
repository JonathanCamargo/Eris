// Eris is a firmware that handles low level control
// It is based on ChibiOS and ported to Arduino


#include "Eris.h"
//#include "sinewave.h"
//#include "serialcommand.h"
//#include "joints.h"
//#include "serialcommand.h"

//TaskHandle_t xHandleThread1 = NULL;
const char firmwareInfo[]=FIRMWARE_INFO;

void Task1(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Sleep for 1000 milliseconds.    
    // Toggle pin to show heartbeat
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));    
    Serial.println("heartbeat task1");        
    vTaskDelay(pdMS_TO_TICKS(300));    
    Serial.println("heartbeat");
  }
}


void start(){
  
  xTaskCreate(Task1, "Task1", 128, NULL, 0, NULL);
  //Error::start(); // Start error notification task (Do not disable)
  
  // start special tasks from extenal sources
  // SineWave::start();
  //Joints::start();
  // Command interfaces 
  //SerialCom::start();
  //SerialInterface::start(); 


  //vTaskStartScheduler(); 
}


void setup(){  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for //Serial port to connect. Needed for native USB
  }
  delay(100);
  // Setup the initial configuration  
  Serial.println("HELLO, This is Eris");
  delay(100);
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT); 
  digitalWrite(PIN_LED,HIGH); 
  delay(1000);
  Serial.println("HELLO, This is Eris 2");
  digitalWrite(PIN_LED,LOW); 
  delay(1000);
  //pinMode(PIN_ERROR,OUTPUT);

  //Start threads
  start();  
}

void loop(){  
  //This is not used since RTOS takes charge
}
