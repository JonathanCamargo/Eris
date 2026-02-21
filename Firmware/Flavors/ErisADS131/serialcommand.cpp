#include "Eris.h"

static Packet packet; 

namespace SerialCom{

  SerialCommand sCmd;

  eris_thread_ref_t readSerial = NULL;
  eris_thread_ref_t streamSerial = NULL;
  static char strbuffer[128]; 
  static bool stream_en=false;

  long startTime = 0;
  
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
    
    sCmd.addCommand("EMG",TransmitEMG); // Transmit the current EMG buffer
    
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.addCommand("SD_REC", StartADC);
    sCmd.addCommand("SD_NREC", StopADC);
    
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

void StartADC(){
  eriscommon::printText("Start collecting on ADC...\n"); 
  ADS131::startCollecting();
}


void StopADC(){
  ADS131::stopCollecting();
  eriscommon::printText("Stop collecting on ADC\n");
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
  eriscommon::printText("Streaming off\n");
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
        Error::RaiseError(COMMAND,(char *)"StreamingSetFeatures");
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

void TransmitEMG(){
   ERIS_CRITICAL_ENTER();
   EMGSample_t samples[MEMBUFFERSIZE];   
   int num=ADS131::buffer.FetchData(samples,(char*)"EMG",MEMBUFFERSIZE);
   long missed=ADS131::buffer.missed();
   ERIS_CRITICAL_EXIT();

   Serial.print("EMG:");
   
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
  // Show the data
   if (num>0){
     uint8_t i=0;
     Serial.print("(@");
     Serial.print(samples[0].timestamp,2);
     Serial.print("ms)");         
     for (i=0;i<num-1;i++){
        Serial.print(samples[i].ch[0],2);
        Serial.print(",");
     }     
     Serial.print(samples[i].ch[0],2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
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
  Error::RaiseError(COMMAND);
  Serial.println("What?\n");
}


}
