#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "sinewave.h"
#include "serialselector.h"
#include "sdcard.h"
#include "streaming.h"

namespace SerialCom{

  SerialCommand sCmd;

  eris_thread_ref_treadSerial = NULL;
  eris_thread_ref_tstreamSerial = NULL;
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
    
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer    
    sCmd.addCommand("EMG",TransmitEMG); // Transmit the current EMG buffer  
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current FSR buffer
    sCmd.addCommand("NEG",SelectNegativeElectrode); // Select the negative electrode lead
    
       

    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.addCommand("SD_REC",StartRecording); //Save to sd card
    sCmd.addCommand("SD_NREC",StopRecording); //Save to sd card
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    eriscommon::println("Serial Commands are ready");

    // create task at priority one
    readSerial=eris_thread_create(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=eris_thread_create(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  char temp[50]; 
  int num = sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  eriscommon::print(temp);  
}

void StartRecording(){
  #if not SDCARD
    eriscommon::printText("SDCARD disabled in configuration.h"); 
  #else
  char * arg = sCmd.next();  
  if (arg != NULL) {
    SDCard::setTrialName(arg);
  } else {
    SDCard::setTrialName(SDCard::DEFAULT_TRIALNAME);
  }
  sprintf(strbuffer, "Start record on SDCARD (trial:%s)", SDCard::getTrialName()); 
  eriscommon::printText(strbuffer); 
  SDCard::StartRecording();

  startTime = micros();
  
  #endif
}


void StopRecording(){
  SDCard::StopRecording();
  eriscommon::printText("Stop record on SDCard");
}


void StartThreads(){
  eriscommon::println("Starting threads manually");
  KillThreads(); 
}

void StreamingStart(){  
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;
  eriscommon::println("Streaming off");
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
  eriscommon::println("Features Ready");   
}

void KillThreads(){
  eriscommon::println("Killing threads");  
}

void stream(){
  Streaming::Stream();
}

void LED_on() {
  eriscommon::println("LED on");
  digitalWrite(PIN_LED, HIGH);
}

void LED_off() {
  eriscommon::println("LED off");
  digitalWrite(PIN_LED, LOW);
}

void TransmitSineWave(){
   ERIS_CRITICAL_ENTER();
   float values[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(values,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   ERIS_CRITICAL_EXIT();

   eriscommon::print("SineWave:");
   
   // Show number of missed points
   eriscommon::print("(");
   eriscommon::print(missed);
   eriscommon::print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     for (i=0;i<num-1;i++){
        eriscommon::print(values[i],2);
        eriscommon::print(",");
     }
     eriscommon::println(values[i],2);
   } else {
     eriscommon::println();
   }
}

void SelectNegativeElectrode(){

    ERIS_CRITICAL_ENTER();
    char *arg;
    arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
    if (arg != NULL) {    // As long as it existed, take it
      uint8_t index=atoi(arg);
      SerialSelector::SelectNegativeElectrode(index);           
    }
    ERIS_CRITICAL_EXIT();   
}


void TransmitEMG(){
   ERIS_CRITICAL_ENTER();
   float values[MEMBUFFERSIZE];   
   int num=EMG::buffer[0]->FetchData(values,(char*)"EMG",MEMBUFFERSIZE);
   long missed=EMG::buffer[0]->missed();   
   ERIS_CRITICAL_EXIT();

   eriscommon::print("EMG:");
   
   // Show number of missed points
   eriscommon::print("(");
   eriscommon::print(missed);
   eriscommon::print(") ");
   // Show the data
   if (num>0){
      uint8_t i=0;
      for (i=0;i<num-1;i++){
        eriscommon::print(values[i],2);
        eriscommon::print(",");
      }
      eriscommon::println(values[i],2);
   } else {
      eriscommon::println();
   }
}


void TransmitFSR(){
   //eriscommon::print("ENTERED"); 
   ERIS_CRITICAL_ENTER();
   float values[MEMBUFFERSIZE];      
   int num=FSR::buffer[0]->FetchData(values,(char*)"FSR",MEMBUFFERSIZE);
   long missed=FSR::buffer[0]->missed();   
   ERIS_CRITICAL_EXIT();

   eriscommon::print("FSR:");
   
   // Show number of missed points
   eriscommon::print("(");
   eriscommon::print(missed);
   eriscommon::print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     for (i=0;i<num-1;i++){
        eriscommon::print(values[i],2);
        eriscommon::print(",");
     }
     eriscommon::println(values[i],2);
   } else {
      eriscommon::println();
   }
}

void SayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    eriscommon::print("Hello ");
    eriscommon::println(arg);
  }
  else {
    eriscommon::println("Hello!");
  }
}


void GetID() {
  eriscommon::print("Firmware:");
  eriscommon::println(FIRMWARE_INFO);

}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Error::RaiseError(COMMAND);
  eriscommon::println("What?");
}


}
