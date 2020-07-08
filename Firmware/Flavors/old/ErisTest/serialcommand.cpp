#include "Eris.h"
#include "serialcommand.h"


#include "sdcard.h"

#include "streaming.h"

#include "emg.h"
#include "fsr.h"
#include "sinewave.h"


namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  static bool stream_en=false; 
  static char strbuffer[128];

  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 1024);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(100);
    }
	}

  //To process //Serial commands
  static THD_WORKING_AREA(waStreamSerial_T, 4096);
  static THD_FUNCTION(StreamSerial_T, arg) {    
    while(1){
      if (stream_en){
        stream();  
      }                        
      chThdSleepMilliseconds(STREAMING_PERIOD_MS);      
    }
  }
 
  /***************************************************************/

	void start(void){ 

    //COMMANDS:
    sCmd.addCommand("INFO",Info);       // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer    
    sCmd.addCommand("EMG",TransmitEMG); // Transmit the current EMG buffer    
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current EMG buffer    

    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.addCommand("SD_REC",StartRecording); //Save to sd card
    sCmd.addCommand("SD_NREC",StopRecording); //Save to sd card

    sCmd.addCommand("TPAC_ON",enableTPackets); //Set prints as TPackets
    sCmd.addCommand("TPAC_OFF",disableTPackets); // Set prints as Serial.print
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    eriscommon::printText("Serial Commands are ready");

  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}

void Info(){
  char temp[56]; 
  sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  eriscommon::printText(temp);   
}

void enableTPackets() {
  eriscommon::setT(true); 
  sprintf(strbuffer, "T packets flag:\t%d", eriscommon::getT()); 
  eriscommon::printText(strbuffer); 
}

void disableTPackets() {  
  eriscommon::setT(false); 
  sprintf(strbuffer, "T packets flag:\t%d", eriscommon::getT()); 
  eriscommon::printText(strbuffer); 
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
  eriscommon::setT(true);
}

void StreamingStop(){
  stream_en=false;
  eriscommon::setT(false);
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
  digitalWrite(PIN_LED,HIGH);   
}

void stream(){
  Streaming::Stream();
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
  sprintf(strbuffer,"Start record on SDCard (trial:%s)",SDCard::trialname);  
  eriscommon::printText(strbuffer);
  SDCard::StartRecording();  
  #endif
}

void StopRecording(){  
  SDCard::StopRecording();
  eriscommon::printText("Stop record on SDCard");
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

void TransmitEMG(){
   chSysLockFromISR();
   
   float values[MEMBUFFERSIZE];   
   int num=EMG::buffer[0]->FetchData(values,(char*)"EMG",MEMBUFFERSIZE);
   long missed=EMG::buffer[0]->missed();   
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
   chSysLockFromISR();      
   float values[MEMBUFFERSIZE];   
   int num=FSR::buffer[0]->FetchData(values,(char*)"FSR",MEMBUFFERSIZE);
   long missed=FSR::buffer[0]->missed();   
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
  eriscommon::printText("What?");
}

}
