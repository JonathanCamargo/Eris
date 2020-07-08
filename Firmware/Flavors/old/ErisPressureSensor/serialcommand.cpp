#include "Eris.h"
#include "serialcommand.h"
#include "sinewave.h"
#include "pressure.h"
#include "sdcard.h"
#include "streaming.h"

//#include <math.h>

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  static bool stream_en=false;

  #if DEBUG_TIME
    time_measurement_t time;
  #endif

  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 4096);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(200);
    }
	}

  //To process //Serial commands
  static THD_WORKING_AREA(waStreamSerial_T, 4096);
  static THD_FUNCTION(StreamSerial_T, arg) {
    while(1){
      if (stream_en){
        stream();  
      }                 
      chThdSleepMilliseconds(40);
    }
  }
 
  /***************************************************************/


	void start(void){

    #if DEBUG_TIME
    chTMObjectInit (&time);
    #endif

    //COMMANDS:
     sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off

    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("P",TransmitPressure); // Transmit the current pressure buffer

    #if FEATURES
    sCmd.addCommand("SINE_RMS",TransmitSineWaveRMS); // Transmit the current sinewave buffer (experimental)
    #endif
    
    sCmd.addCommand("TIME",test); // Transmit the current joint state
   
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.addCommand("REC",StartRecording); //Save to sd card
    sCmd.addCommand("STOPREC",StopRecording); //Save to sd card

    //sCmd.addCommand("PID",SetPID);  // Modify pid parameters  TODO
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO, StreamSerial_T, NULL);

	}


 void INFO() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);
}

void StartRecording(){
  Serial.println("Start record on SDCard");
  SDCard::StartRecording();
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


void TransmitPressure(){
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=Pressure::buffer.FetchData(values,(char*)"PRESSURE",MEMBUFFERSIZE);
   long missed=Pressure::buffer.missed();   
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


#if FEATURES
   
void TransmitSineWaveRMS(){
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=SineWave::rmsbuffer.FetchData(values,(char*)"SINEWAVE",MEMBUFFERSIZE);
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
#endif

void test(){
     #if DEBUG_TIME
      chSysLockFromISR();
      Serial.print("Last:");Serial.print(I2CInterface::time.last/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("Best:");Serial.print(I2CInterface::time.best/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("Worst:");Serial.print(I2CInterface::time.worst/DEBUG_SYSCLK);Serial.println("(us) ");
      Serial.print("n=");Serial.print(I2CInterface::time.n);Serial.println("samples");
      chSysUnlockFromISR();
     #else
      Serial.print("Time debugging is not active");
      #endif
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
  char c[20];
  strcpy(c,command);
  Error::RaiseError(COMMAND,c);
  Serial.println("What?");
}


}
