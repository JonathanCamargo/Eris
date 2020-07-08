#include "Eris.h"
#include "serialcommand.h"


#include "imu.h"
#include "fsr.h"
#include "sinewave.h"
#include "sync.h"

#include "sdcard.h"
#include "streaming.h"

//Static allocation to avoid moving a lot of memory in RTOS
static FSRSample_t fsrsamples[MEMBUFFERSIZE];   
static boolSample_t syncsamples[MEMBUFFERSIZE];
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
    sCmd.addCommand("IMU",TransmitIMU); // Transmit the current IMU buffer  
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current FSR buffer
    sCmd.addCommand("SYNC",TransmitSync); // Transmit the current SYNC buffer  
    sCmd.addCommand("FAIL",ShowFailures); // Configure the streaming functions

    sCmd.addCommand("S_TIME",SynchronizeTime); //Synchronize time
           
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming
    
    sCmd.addCommand("F_ON", EnableFeatures);   //  Enable feature extraction and transmission
    sCmd.addCommand("F_OFF", DisableFeatures); //  Disable feature extraction and transmission
    sCmd.addCommand("F_R",Register );          //  Register module extractors into a features helper
    sCmd.addCommand("F_MASK",ChangeMask );     //  Disable feature extraction and transmission
    sCmd.addCommand("F_INFO",FeaturesInfo);    //  Display information for all the features helpers
    /*
    * NOTE:
    * This is how the driver communicates with the teensy. Before enabling feature extraction (F_ON), masks must be set using F_MASK. Once features are enabled, the teensy will continuously stream computed features
    * for regression. The rate of this is determined by the regression increment. To change the window, increment, or ambulation mode used to calculate regression features, use REG [NEW WINDOW] [NEW INCREMENT] [NEW MODE]. 
    * To make a query for classification features, use CLASS [GAIT LOCATION] [NEXT WINDOW]. Features will be extracted using [GAIT LOCATION] and the current window, and the window size will only be updated once features 
    * have been extracted. 
    */
    
 
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
  long test=chUnusedThreadStack(waStreamSerial_T,sizeof(waStreamSerial_T));
  Serial.println("------");
  Serial.print(test);
  Serial.println("------");
}

void ShowFailures(){  
  Serial.println(IMU::failures);
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
        imubuffer=&SDCard::imutrunkbuffer;
        break;
      case 1:
        imubuffer=&SDCard::imuthighbuffer;
        break;
      case 2:
        imubuffer=&SDCard::imushankbuffer;
        break;
      case 3:
        imubuffer=&SDCard::imufootbuffer;
        break;
      default:
        imubuffer=&SDCard::imutrunkbuffer;
        break;
    }    
  }
  else {
      imubuffer=&SDCard::imutrunkbuffer;
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


void TransmitSync(){
  chSysLockFromISR();
  boolSample_t *samples=&syncsamples[0];
  int num=Sync::buffer.FetchData(samples,(char*)"SYNC",MEMBUFFERSIZE);
  long missed=Sync::buffer.missed();   
  chSysUnlockFromISR();   
  Serial.print("Sync:");   
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
