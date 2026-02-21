#include "Eris.h"
#include "serialcommand.h"

#include "motor.h"
#include "potentiometer.h"
#include "sinewave.h"
#include "streaming.h"


char strbuffer[STRBUFFERSIZE];

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

  //To process //Serial commands
  static THD_WORKING_AREA(waStreamSerial_T, 1024);
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
    eriscommon::setPrintPacketMode(false);
    //COMMANDS:
    sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer    
    sCmd.addCommand("POT",TransmitPotentiometer); // Transmit the current sinewave buffer    
    sCmd.addCommand("MOT_F",Motor::Forward); // Motor forward
    sCmd.addCommand("MOT_B",Motor::Backward); // Motor forward
    sCmd.addCommand("MOT_STOP",Motor::Break); // Motor idle
    sCmd.addCommand("MOT_OFF",Motor::Idle); // Motor forward
    
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_TIME",SynchronizeTime); //Synchronize time       
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    eriscommon::println("Serial Commands are ready");

    // create task at priority one
    readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  char temp[50]; 
  sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  eriscommon::print(temp);  
}

void StartThreads(){
  eriscommon::print("Starting threads manually");  
  KillThreads(); 
}

void StreamingStart(){  
  eriscommon::setPrintPacketMode(true);
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;  
  eriscommon::print("Streaming off");  
  eriscommon::setPrintPacketMode(false);
}

void SynchronizeTime(){  
  // Reset the start time
  chSysLockFromISR();
  t0=micros();
  chSysUnlockFromISR();
}


void KillThreads(){
  eriscommon::print("Killing threads");  
}

void stream(){
  Streaming::Stream();
}

void LED_on() {
  eriscommon::print("LED on");
  digitalWrite(PIN_LED, HIGH);
}

void LED_off() {
  eriscommon::print("LED off");
  digitalWrite(PIN_LED, LOW);
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
  eriscommon::print("Features Ready");   
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


void TransmitPotentiometer(){
   chSysLockFromISR();
   PotentiometerSample_t samples[MEMBUFFERSIZE];
   int num=Potentiometer::buffer.FetchData(samples,(char*)"POTENTIOMETER",MEMBUFFERSIZE);
   long missed=Potentiometer::buffer.missed();
   chSysUnlockFromISR();
   eriscommon::print("Potentiometer:");
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
