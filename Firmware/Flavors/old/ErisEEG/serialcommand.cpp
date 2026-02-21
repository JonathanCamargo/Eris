#include "Eris.h"
#include "serialcommand.h"
#include "sdcard.h"
#include "eeg.h"
#include "fsr.h"
#include "sinewave.h"

#include "streaming.h"

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  static bool stream_en=false;

  
  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 1024);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(100);
    }
	}

  //To stream data
  static THD_WORKING_AREA(waStreamSerial_T, 4096);
  static THD_FUNCTION(StreamSerial_T, arg) {    
    while(1){
      if (stream_en){
        stream();  
      }                        
      chThdSleepMilliseconds(40); //25Hz
    }
  }
 
  /***************************************************************/

	void start(void){ 

    //COMMANDS:
    sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("EMG",TransmitEEG); // Transmit the current EEG buffer            
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current FSR buffer            
       
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads
    sCmd.addCommand("START",StartThreads);//Start threads

    sCmd.addCommand("SD_REC",StartRecording); //Save to sd card
    sCmd.addCommand("SD_NREC",StopRecording); //Save to sd card


     sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+1, StreamSerial_T, NULL);

	}


void INFO() {  
  char temp[56]; 
  sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  Serial.print(temp);   
}

void StartThreads(){
  Serial.println("Starting threads manually");
  KillThreads(); 
}

void KillThreads(){
  Serial.println("Killing threads");  
}

void StreamingStart(){  
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;
  Serial.println("Streaming off");
}

void StreamingSetFeatures(){   
  char *arg;
  chSysLockFromISR();
  Streaming::ClearFunctions();
  // Select the streaming function based on names
  arg = sCmd.next();
  while(arg!= NULL){    
      bool found=Streaming::AddFunction(arg);
      if (!found){
        Error::RaiseError(COMMAND,(char *)"StreamingSetFeatures");
        Streaming::ClearFunctions();
        chSysUnlockFromISR();        
        return;
      }
      arg = sCmd.next();
  }      
  chSysUnlockFromISR();
  Serial.println("Features Ready");   
}


void stream(){
  Streaming::Stream();
}

void StartRecording(){
  char * arg = sCmd.next();  
  if (arg != NULL) {
    SDCard::setTrialName(arg);
  } else {
    SDCard::setTrialName(SDCard::DEFAULT_TRIALNAME);
  }
  Serial.println("Start record on SDCard");
  SDCard::StartRecording();  
}

void StopRecording(){
  SDCard::StopRecording();
  Serial.println("Stop record on SDCard");
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
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(values,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   chSysUnlockFromISR();
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num<=0){
    return;
   }
   uint8_t i=0;
   for (i=0;i<num-1;i++){
      Serial.print(values[i],2);
      Serial.print(",");
   }
   Serial.println(values[i],2);
}


void TransmitEEG(){
   //Display ch0 data
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=EEG::buffer[0]->FetchData(values,(char*)"EEG",MEMBUFFERSIZE);
   long missed=EEG::buffer[0]->missed();   
   chSysUnlockFromISR();
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num<=0){
    return;
   }
   uint8_t i=0;
   for (i=0;i<num-1;i++){
      Serial.print(values[i],2);
      Serial.print(",");
   }
   Serial.println(values[i],2);
}



void TransmitFSR(){
   //Display ch0 data
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=FSR::buffer.FetchData(values,(char*)"FSR",MEMBUFFERSIZE);
   long missed=FSR::buffer.missed();   
   chSysUnlockFromISR();
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num<=0){
    return;
   }
   uint8_t i=0;
   for (i=0;i<num-1;i++){
      Serial.print(values[i],2);
      Serial.print(",");
   }
   Serial.println(values[i],2);
}

void GetID() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);

}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  char c[24];
  strcpy(c,command);
  Error::RaiseError(COMMAND,c);
  Serial.println("What?");
}


}
