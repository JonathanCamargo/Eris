#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "sinewave.h"
#include "sync.h"
#include "streaming.h"

//Static allocation to avoid moving a lot of memory in RTOS
static FSRSample_t fsrsamples[MEMBUFFERSIZE];
static uint8_tSample_t syncsamples[MEMBUFFERSIZE];


namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  static bool stream_en=false;

  
  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 4096);
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
      chThdSleepMilliseconds(8);
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
    sCmd.addCommand("SYNC",TransmitSync); // Transmit the current EMG buffer  
      

    sCmd.addCommand("S_T0",ResetT0); // Restart t0 to 0
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
  
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO, StreamSerial_T, NULL);

	}

void Info(){
   Serial.print("Firmware:");
   Serial.println(FIRMWARE_INFO);
}

void ResetT0(){
  t0=0;
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
   floatSample_t samples[MEMBUFFERSIZE];
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();
   chSysUnlockFromISR();
   eriscommon::print("SineWave:");
   // Show number of missed points
   eriscommon::print("(missed:");
   eriscommon::print(missed);
   eriscommon::print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     eriscommon::print("(@");
     eriscommon::print(samples[0].timestamp,2);
     eriscommon::print("ms)");
     for (i=0;i<num-1;i++){
        eriscommon::print(samples[i].value,2);
        eriscommon::print(",");
     }
     eriscommon::print(samples[i].value,2);
     eriscommon::print("(@");
     eriscommon::print(samples[i].timestamp,2);
     eriscommon::println("ms)");
   }
   else {
     eriscommon::println("num < 0");
   }
}
 

void TransmitEMG(){
   chSysLockFromISR();
   
   EMGSample_t values[MEMBUFFERSIZE];   
   int num=EMG::buffer.FetchData(values,(char*)"EMG",MEMBUFFERSIZE);
   long missed=EMG::buffer.missed();   
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
      Serial.print(values[i].ch[0],2);
      Serial.print(",");
   }
   Serial.println(values[i].ch[0],2);

}


void TransmitFSR(){
  chSysLockFromISR();
  FSRSample_t *samples=&fsrsamples[0];
  int num=FSR::buffer.FetchData(samples,(char*)"FSR",MEMBUFFERSIZE);
  long missed=FSR::buffer.missed();
  chSysUnlockFromISR();
  eriscommon::print("FSR[ch0]:");
  // Show number of missed points
  eriscommon::print("(missed:");
  eriscommon::print(missed);
  eriscommon::print(") ");
  // Show the data
   if (num>0){
     uint8_t i=0;
     eriscommon::print("(@");
     eriscommon::print(samples[0].timestamp,2);
     eriscommon::print("ms)");
     for (i=0;i<num-1;i++){
        eriscommon::print(samples[i].ch[0],2);
        eriscommon::print(",");
     }
     eriscommon::print(samples[i].ch[0],2);
     eriscommon::print("(@");
     eriscommon::print(samples[i].timestamp,2);
     eriscommon::println("ms)");
   }
   else {
     eriscommon::println("num < 0");
   }
}



void TransmitSync(){
   chSysLockFromISR();      
   uint8_tSample_t samples[MEMBUFFERSIZE];   
   int num=Sync::buffer.FetchData(samples,(char*)"FSR",MEMBUFFERSIZE);
   long missed=Sync::buffer.missed();   
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
      Serial.print("(");      
      Serial.print(samples[i].timestamp,2);
      Serial.print(")");
      Serial.print(samples[i].value);
      Serial.print(",");
   }
   Serial.print("(");      
   Serial.print(samples[i].timestamp,2);
   Serial.print(")");
   Serial.println(samples[i].value);
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
