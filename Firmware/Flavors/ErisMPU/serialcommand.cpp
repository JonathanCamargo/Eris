#include "Eris.h"
#include "serialcommand.h"


#include "imu.h"
#include "fsr.h"
#include "sinewave.h"

#include "streaming.h"

//Static allocation to avoid moving a lot of memory in RTOS
static FSRSample_t fsrsamples[MEMBUFFERSIZE];   
static IMUSample_t imusamples[MEMBUFFERSIZE];

char strbuffer[STRBUFFERSIZE];

namespace SerialCom{

  SerialCommand sCmd;
  
  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;
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
  static THD_WORKING_AREA(waStreamSerial_T, 2048);
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
    sCmd.addCommand("INITIMU",InitIMU); // Transmit the current IMU buffer  
    sCmd.addCommand("IMU",TransmitIMU); // Transmit the current IMU buffer  
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current FSR buffer
    sCmd.addCommand("FAIL",ShowFailures); // Configure the streaming functions

    sCmd.addCommand("TIME0",SynchronizeTime); //Synchronize time
           
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming
             
    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
  
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

    // create task at priority one
    readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  char temp[32]; 
  sprintf(temp, "Firmware: %s", FIRMWARE_INFO);   
  eriscommon::println(temp);
}

void ShowFailures(){  
  eriscommon::println(IMU::failures);
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
  t0=micros();
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
        Error::RaiseError(Error::COMMAND,(char *)"StreamingSetFeatures");
        Streaming::ClearFunctions();
        chSysUnlockFromISR();        
        return;
      }
      arg = sCmd.next();
  }      
  chSysUnlockFromISR();
  eriscommon::println("Features Ready");   
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

void InitIMU(){
  IMU::InitIMU();
}
void TransmitSineWave(){
   chSysLockFromISR();
   floatSample_t samples[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   chSysUnlockFromISR();   
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

void TransmitIMU(){
  char *arg;
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  ErisBuffer<IMUSample_t> * imubuffer;
  if (arg != NULL) {    // As long as it existed, take it
    Serial.print("IMU[");
    Serial.print(arg);
    Serial.print("] ");
    int imuidx=atoi(arg);
    switch (imuidx){
      case 0: 
        imubuffer=&IMU::bufferTrunk;
        break;
      case 1:
        imubuffer=&IMU::bufferThigh;
        break;
      case 2:
        imubuffer=&IMU::bufferShank;
        break;
      case 3:
        imubuffer=&IMU::bufferFoot;
        break;
      default:
        imubuffer=&IMU::bufferTrunk;
        break;
    }    
  }
  else {
      imubuffer=&IMU::bufferTrunk;
  }
      
  chSysLockFromISR();
  IMUSample_t *samples=&imusamples[0];
  int num=imubuffer->FetchData(samples,(char*)"IMU",MEMBUFFERSIZE);  
  long missed=imubuffer->missed();   
  chSysUnlockFromISR();     
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
        Serial.print(samples[i].ax,2);
		Serial.print(":");
		Serial.print(samples[i].ay,2);
		Serial.print(":");
		Serial.print(samples[i].az,2);		
        Serial.print(",");
     }          
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
   }           
}

void TransmitFSR(){    
  chSysLockFromISR();
  FSRSample_t *samples=&fsrsamples[0];
  int num=FSR::buffer.FetchData(samples,(char*)"FSR",MEMBUFFERSIZE);
  long missed=FSR::buffer.missed();   
  chSysUnlockFromISR();   
  Serial.print("FSR[ch0]:");   
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
  Error::RaiseError(Error::COMMAND);
  Serial.println("What?");
}


}
