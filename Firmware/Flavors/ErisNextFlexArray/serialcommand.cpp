#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "sinewave.h"
#include "serialselector.h"
#include "sdcard.h"
#include "streaming.h"

static Packet packet; 

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;
  static char strbuffer[128]; 
  static bool stream_en=false;

  long startTime = 0;

  
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
  static THD_WORKING_AREA(waStreamSerial_T, 1024);
  static THD_FUNCTION(StreamSerial_T, arg) {    
    while(1){
      if (stream_en){
        stream();  
      }                        
      chThdSleepMilliseconds(10);
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
    Serial.println("Serial Commands are ready");

    // create task at priority one
    readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  packet.start(Packet::PacketType::TEXT); 
  char temp[50]; 
  int num = sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  //Serial.print(temp); 
  packet.append((uint8_t *)temp, num); 
  packet.send(); 
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
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(values,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   chSysUnlockFromISR();

   Serial.print("SineWave:");
   
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     for (i=0;i<num-1;i++){
        Serial.print(values[i],2);
        Serial.print(",");
     }
     Serial.println(values[i],2);
   } else {
     Serial.println();
   }
}

void SelectNegativeElectrode(){

    chSysLockFromISR();
    char *arg;
    arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
    if (arg != NULL) {    // As long as it existed, take it
      uint8_t index=atoi(arg);
      SerialSelector::SelectNegativeElectrode(index);           
    }
    chSysUnlockFromISR();   
}


void TransmitEMG(){
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=EMG::buffer[0]->FetchData(values,(char*)"EMG",MEMBUFFERSIZE);
   long missed=EMG::buffer[0]->missed();   
   chSysUnlockFromISR();

   Serial.print("EMG:");
   
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num>0){
      uint8_t i=0;
      for (i=0;i<num-1;i++){
        Serial.print(values[i],2);
        Serial.print(",");
      }
      Serial.println(values[i],2);
   } else {
      Serial.println();
   }
}


void TransmitFSR(){
   //Serial.print("ENTERED"); 
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];      
   int num=FSR::buffer[0]->FetchData(values,(char*)"FSR",MEMBUFFERSIZE);
   long missed=FSR::buffer[0]->missed();   
   chSysUnlockFromISR();

   Serial.print("FSR:");
   
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     for (i=0;i<num-1;i++){
        Serial.print(values[i],2);
        Serial.print(",");
     }
     Serial.println(values[i],2);
   } else {
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
  Serial.println("What?");
}


}
