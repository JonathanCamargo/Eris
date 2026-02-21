#include "Eris.h"

char strbuffer[STRBUFFERSIZE];

namespace SerialCom{

SerialCommand sCmd;  

//////////////////////////
// Task_ReadSerial: monitor serial0 port for commands and respond 
// on callback functions.
void Task_ReadSerial(void *pvParameters)  // This is a task.
{
    (void) pvParameters;    
    while(1){
      sCmd.readSerial();
      vTaskDelay(pdMS_TO_TICKS(READSERIAL_PERIOD_MS)); // wait for one second
    }
}
  
void start(void){ 
  ////////////////////
  // COMMANDS:
  // User can add more commands as desired. Each command executes a given function
  sCmd.addCommand("INFO",INFO);  // Display information about the firmware    
  sCmd.addCommand("HELLO",SayHello);  // Display information about the firmware    
  sCmd.addCommand("SHOW_DATA",ShowDataGeneratorSamples);  // Display information about the firmware    
  sCmd.addCommand("X",XParser);
  sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
  Console.println("Serial Commands are ready");

  // create task at priority one      
  xTaskCreate(
    Task_ReadSerial
    ,  "ReadSerial"   // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  }

void XParser(){
  Console.println("moving hand");
  char *arg;
  int values[5]={0,0,0,0,0};
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  uint8_t i=0;
  while (arg!=NULL){
    values[i]=atoi(arg);
    arg = sCmd.next();
    i++;
  }
  Servos::move(values);
}

void INFO() {    
  int num = sprintf(strbuffer, "Firmware: %s", FIRMWARE_INFO);     
  Console.println(strbuffer);    
}

void SayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    Console.print("Hello ");
    Console.println(arg);
  }
  else {
    Console.println("Hello!");
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  //Error::RaiseError(COMMAND);
  Console.println("What?");
}

void ShowDataGeneratorSamples(){
  floatSample_t samples[MEMBUFFERSIZE];
  int num=DataGenerator::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
  long missed=DataGenerator::buffer.missed();     
  Console.print("DataGeneratorSamples:");   
  // Show number of missed points
  Console.print("(missed:");
  Console.print(missed,DEC);
  Console.print(") ");  
  Console.print("num:");
  Console.println(num,DEC);
  // Show the data     
  for (int i=0;i<num;i++){
    DataGenerator::PrintSample(samples[i]);
  }       
}
}
