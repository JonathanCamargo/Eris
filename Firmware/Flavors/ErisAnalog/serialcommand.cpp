#include "Eris.h"
#include "serialcommand.h"
#include "sinewave.h"
#include "analog.h"
#include "streaming.h"

namespace SerialCom{

  SerialCommand sCmd;

  eris_thread_ref_t readSerial = NULL;
  eris_thread_ref_t streamSerial = NULL;
  static bool stream_en=false;

  long startTime = 0;

  //Static allocation to avoid moving a lot of memory in RTOS
   AnalogSample_t analogSamples[MEMBUFFERSIZE];      


  
  /********************** Threads *********************************/
  //To process //Serial commands
  ERIS_THREAD_WA(waReadSerial_T, 1024);
	ERIS_THREAD_FUNC(ReadSerial_T) {
    while(1){
		  sCmd.readSerial();
		  eris_sleep_ms(100);
    }
	}

  //To process //Serial commands
  ERIS_THREAD_WA(waStreamSerial_T, 1024);
  ERIS_THREAD_FUNC(StreamSerial_T) {
    while(1){
      if (stream_en){
        stream();
      }
      eris_sleep_ms(10);
    }
  }
 
  /***************************************************************/

	void start(void){ 

    //COMMANDS:
    sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer    
    sCmd.addCommand("A",TransmitAnalog); // Transmit the current EMG buffer      
  
    sCmd.addCommand("TIME0",SynchronizeTime); // Synchronize time                 
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

    // create task at priority one
    readSerial=eris_thread_create(waReadSerial_T, 1024, NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=eris_thread_create(waStreamSerial_T, 1024, NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  packet.start(Packet::PacketType::TEXT); 
  char temp[50]; 
  int num = sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  //Serial.print(temp); 
  packet.append((uint8_t *)temp, num); 
  packet.send(); 
}

void StartThreads(){
  Serial.println("Starting threads manually");
  KillThreads(); 
}

void StreamingStart(){  
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;
  Serial.println("Streaming off");
}

void SynchronizeTime(){  
  // Reset the start time
  ERIS_CRITICAL_ENTER();
  t0=micros();
  ERIS_CRITICAL_EXIT();
}

void StreamingSetFeatures(){   
  char *arg;
  ERIS_CRITICAL_ENTER();
  Streaming::ClearFunctions();
  // Select the streaming function based on names
  arg = sCmd.next();
  while(arg!= NULL){    
      bool found=Streaming::AddFunction(arg);
      if (!found){
        Error::RaiseError(Error::COMMAND,(char *)"StreamingSetFeatures");
        Streaming::ClearFunctions();
        ERIS_CRITICAL_EXIT();        
        return;
      }
      arg = sCmd.next();
  }      
  ERIS_CRITICAL_EXIT();
  Serial.println("Features Ready");   
}

void KillThreads(){
  Serial.println("Killing threads");  
}

void stream(){
  Streaming::Stream();
}

void LED_on() {
  Serial.println("LED on");
  digitalWrite(PIN_LED, HIGH);
}

void LED_off() {
  Serial.println("LED off");
  digitalWrite(PIN_LED, LOW);
}

void TransmitSineWave(){
   ERIS_CRITICAL_ENTER();
   floatSample_t samples[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   ERIS_CRITICAL_EXIT();   
   Serial.print("SineWave:");   
   // Show number of missed points
   Serial.print("(missed:");
   Serial.print(missed);
   Serial.print(") ");   
   // Show the data
   if (num>0){
     uint8_t i=0;
     Serial.print("(@");
     Serial.print(samples[0].timestamp,2);
     Serial.print("ms)");         
     for (i=0;i<num-1;i++){
        Serial.print(samples[i].value,2);
        Serial.print(",");
     }     
     Serial.print(samples[i].value,2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
   }         
}

void TransmitAnalog(){
   ERIS_CRITICAL_ENTER();
   int num=Analog::buffer.FetchData(analogSamples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=Analog::buffer.missed();   
   ERIS_CRITICAL_EXIT();   
   Serial.print("Analog:");   
   // Show number of missed points
   Serial.print("(missed:");
   Serial.print(missed);
   Serial.print(") ");   
   // Show the data
   if (num>0){
     uint8_t i=0;
     Serial.print("(@");
     Serial.print(analogSamples[0].timestamp,2);
     Serial.print("ms)");         
     for (i=0;i<num-1;i++){
        Serial.print(analogSamples[i].ch[0],2);
        Serial.print(",");
     }     
     Serial.print(analogSamples[i].ch[0],2);
     Serial.print("(@");
     Serial.print(analogSamples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
   }         
}
  

void SayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    Serial.print("Hello ");
    Serial.println(arg);
  }
  else {
    Serial.println("Hello!");
  }
}


void GetID() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);

}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Error::RaiseError(Error::COMMAND);
  Serial.println("What?");
}


}
