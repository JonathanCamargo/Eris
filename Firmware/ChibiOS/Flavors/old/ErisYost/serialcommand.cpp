#include "Eris.h"
#include "serialcommand.h"

#include "emg.h"
#include "imu.h"
#include "features.h"
#include "sdcard.h"
#include "sinewave.h"
#include "streaming.h"


namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  bool stream_en=false;
  static float values[MEMBUFFERSIZE];   
  
  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 1024);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(150);
    }
	}

  //To process //Serial commands
  static THD_WORKING_AREA(waStreamSerial_T, 1024);
  static THD_FUNCTION(StreamSerial_T, arg) {  
    // Send data using streaming method  
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
    sCmd.addCommand("IMU",TransmitIMU); // Transmit the current EMG buffer
    sCmd.addCommand("FEAT",TransmitFEAT); // Transmit the current feature vector

    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.addCommand("SD_REC",StartRecording); //Save to sd card
    sCmd.addCommand("SD_NREC",StopRecording); //Save to sd card

    sCmd.addCommand("T",TimeInfo); //Save to sd card


    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

    //Attach interrupts for starting / stopping streaming
    attachInterrupt(STEE
    
  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+1, StreamSerial_T, NULL);

	}

void INFO() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);
}

void StartRecording(){
  char * arg = sCmd.next();
  Serial.println("Start record on SDCard");
  if (arg != NULL) {
    strcpy(SDCard::filename,arg);
  } else {
    strcpy(SDCard::filename,SDCard::DEFAULT_FILENAME);
  }
  SDCard::StartRecording();
}

void StartThreads(){
  Serial.println("Starting threads manually");
  KillThreads(); 
}

#define R2US(n) RTC2US(DEBUG_SYSCLK,n)
void TimeInfo(){
 /*sprintf(strbuffer,"(n=%d)\nbest: %1.2f(us)\nworst: %1.2f(us)\nlast: %1.2f(us)\naverage: %1.2f(us)",
 //(int)EMG::t.n,R2US(EMG::t.best),R2US(EMG::t.worst),R2US(EMG::t.last),R2US(EMG::t.cumulative/EMG::t.n)
 );
 //sprintf(strbuffer,"(n=%i)\n",2);
 //chprintf(strbuffer,"hola");
 Serial.println(strbuffer);
 */
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

void TransmitFEAT(){
   // Show number of missed points
   Serial.print("Features:");      
   // Show the data
   uint8_t i=0;
   for (i=0;i<FEATURESSIZE-1;i++){
      Serial.print(Features::lastFeatures[i],2);
      Serial.print(",");
   }
   Serial.println(Features::lastFeatures[i],2);
}


void TransmitEMG(){
   chSysLockFromISR();
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



void TransmitIMU(){
   chSysLockFromISR();
   int num=IMU::buffer[0]->FetchData(values,(char*)"IMU",MEMBUFFERSIZE);
   long missed=IMU::buffer[0]->missed();   
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
  Serial.println(command);
  Error::RaiseError(COMMAND);
  Serial.println("What?");
}


}
