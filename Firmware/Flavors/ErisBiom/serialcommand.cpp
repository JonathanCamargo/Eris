#include "Eris.h"
#include "serialcommand.h"

#include "biom.h"
#include "gait.h"
#include "features.h"
#include "sinewave.h"
#include "sync.h"

#include "streaming.h"

/*
 * NOTE:
 * This is how the driver communicates with the teensy. Before enabling feature extraction (F_ON), masks must be set using F_MASK. Once features are enabled, the teensy will continuously stream computed features
 * for regression. The rate of this is determined by the regression increment. To change the window, increment, or ambulation mode used to calculate regression features, use REG [NEW WINDOW] [NEW INCREMENT] [NEW MODE]. 
 * To make a query for classification features, use CLASS [GAIT LOCATION] [NEXT WINDOW]. Features will be extracted using [GAIT LOCATION] and the current window, and the window size will only be updated once features 
 * have been extracted. 
 */

static Packet packet; 

//Static allocation to avoid moving a lot of memory in RTOS 
static boolSample_t syncsamples[MEMBUFFERSIZE];
static BiomSample_t biomsamples[MEMBUFFERSIZE];

namespace SerialCom{

  SerialCommand sCmd;

  eris_thread_ref_t readSerial = NULL;
  eris_thread_ref_t streamSerial = NULL;
  bool stream_en = false;

  long startTime = 0;
  
  /********************** Threads *********************************/
  //To process //Serial commands
  ERIS_THREAD_WA(waReadSerial_T, 1024);
	ERIS_THREAD_FUNC(ReadSerial_T) {
    while(1){
		  sCmd.readSerial();
		  eris_sleep_ms(10);
    }
	}

  //To stream data
  ERIS_THREAD_WA(waStreamSerial_T, 1024);
  ERIS_THREAD_FUNC(StreamSerial_T) {
    // Send data using streaming method
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
    sCmd.addCommand("TPAC",enableTPackets); //Set prints as TPackets
    
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
    
    sCmd.addCommand("S_TIME", SynchronizeTime);
    
    ////////////// TRANSMIT ////////////////
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("BIOM",TransmitBiom); // Transmit the current Biom buffer
    sCmd.addCommand("GAIT",TransmitGait);
    sCmd.addCommand("SYNC",TransmitSync); // Transmit the current SYNC buffer  

    ////////////// STREAMING ////////////////
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    ///////////// FEATURES /////////////////////
    sCmd.addCommand("F_ON", EnableFeatures);   //  Enable feature extraction and transmission
    sCmd.addCommand("F_OFF", DisableFeatures); //  Disable feature extraction and transmission
    sCmd.addCommand("F_CLASS", ClassifQuery);   // Get classification features
    sCmd.addCommand("F_R",Register);          //  Register module extractors into a features helper
    sCmd.addCommand("F_MASK",ChangeMask);     //  Disable feature extraction and transmission
    sCmd.addCommand("F_WIN", ChangeWindow);   // Change the window
    sCmd.addCommand("F_IDX", ChangeIdx);      // Change the index of a featurehelper array
    sCmd.addCommand("F_INFO",FeaturesInfo);    //  Display information for all the features helpers

    ////////////// DEFAULTS ///////////////////
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    eriscommon::printText("Serial Commands are ready");

  	// create task at priority one
		readSerial=eris_thread_create(waReadSerial_T, 1024,NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=eris_thread_create(waStreamSerial_T, 1024,NORMALPRIO+3, StreamSerial_T, NULL);
	}
 
  void enableTPackets() {
    char* arg;
    arg = sCmd.next();
    if (arg != NULL) {
      eriscommon::setT(atoi(arg) > 0);
      sprintf(strbuffer, "T packets flag:\t%d", eriscommon::getT());
      eriscommon::printText(strbuffer);
    }
  }

  void EnableFeatures() {
    Features::en_features = true;
  }
  
  void DisableFeatures() {
    Features::en_features = false; 
    Features::clearExtractors();
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
  Serial.println(test);
  Serial.println("------");
}

void StartThreads(){
  eriscommon::printText("Starting threads manually");
  KillThreads(); 
}

void StreamingStart(){
  stream_en = true;
}

void StreamingStop(){
  stream_en = false;
  eriscommon::printText("Streaming off");
}

void SynchronizeTime(){  
  // Reset the start time
  t0=micros();
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
  eriscommon::printText("Features Ready");   
}

void KillThreads(){
  eriscommon::printText("Killing threads");  
}

void stream(){
  Streaming::Stream();
}

void LED_on() {
  eriscommon::printText("LED on");
  digitalWrite(PIN_LED, HIGH);
}

void LED_off() {
  eriscommon::printText("LED off");
  digitalWrite(PIN_LED, LOW);
}

void TransmitSineWave(){
   ERIS_CRITICAL_ENTER();
   floatSample_t samples[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   ERIS_CRITICAL_EXIT();   
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

void TransmitBiom(){    
  ERIS_CRITICAL_ENTER();
  BiomSample_t *samples=&biomsamples[0];
  int num=Biom::buffer.FetchData(samples,(char*)"BIOM",MEMBUFFERSIZE);
  long missed=Biom::buffer.missed();   
  ERIS_CRITICAL_EXIT();   
  Serial.print("Biom[ch0]:");   
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

void TransmitGait(){
   Serial.print("Gait: ");
   Serial.println(Gait::currGait);
}

void TransmitSync(){
  ERIS_CRITICAL_ENTER();
  boolSample_t *samples=&syncsamples[0];
  int num=Sync::buffer.FetchData(samples,(char*)"SYNC",MEMBUFFERSIZE);
  long missed=Sync::buffer.missed();   
  ERIS_CRITICAL_EXIT();   
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
    eriscommon::printText("Hello!");
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println(command);
  Error::RaiseError(COMMAND);
  eriscommon::printText("What?");
}


}
