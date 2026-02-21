#include "Eris.h"
#include "serialcommand.h"

#include "biom.h"
#include "gait.h"
#include "features.h"
#include "sinewave.h"
#include "streaming.h"

/*
 * NOTE:
 * This is how the driver communicates with the teensy. Before enabling feature extraction (F_ON), masks must be set using F_MASK. Once features are enabled, the teensy will continuously stream computed features
 * for regression. The rate of this is determined by the regression increment. To change the window, increment, or ambulation mode used to calculate regression features, use REG [NEW WINDOW] [NEW INCREMENT] [NEW MODE]. 
 * To make a query for classification features, use CLASS [GAIT LOCATION] [NEXT WINDOW]. Features will be extracted using [GAIT LOCATION] and the current window, and the window size will only be updated once features 
 * have been extracted. 
 */

#include <FeatureExtractor.h>

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  bool stream_en = false;
  bool classifQuery = false;
  static BiomSample_t biomsamples[MEMBUFFERSIZE];  
  
  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 1024);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(10);
    }
	}

  //To stream data
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
    sCmd.addCommand("TPAC",enableTPackets); //Set prints as TPackets
    
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
    
    sCmd.addCommand("S_TIME", SynchronizeTime);
    sCmd.addCommand("test",test); // TODO DELETE THIS Just for testing
    
    ////////////// TRANSMIT ////////////////
    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("BIOM",TransmitBiom); // Transmit the current Biom buffer
    sCmd.addCommand("GAIT",TransmitGait);
    sCmd.addCommand("FEAT",TransmitFEAT); // Transmit the current feature vector

    ////////////// STREAMING ////////////////
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    ///////////// FEATURES /////////////////////
    sCmd.addCommand("CLASS", getClassificationFeatures);
    sCmd.addCommand("REG", updateRegVals);
    sCmd.addCommand("F_ON", enableFeatures);
    sCmd.addCommand("F_OFF", disableFeatures);

    ////////////// DEFAULTS ///////////////////
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    eriscommon::printText("Serial Commands are ready");

  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);


    t0 = micros();
	}

  void SynchronizeTime() {
    ResetTime();
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

  void enableFeatures() {
    Features::en_features = true;
  }
  
  void disableFeatures() {
    Features::en_features = false; 
    Features::clearExtractors();
  }

  //takes 3 arguments. arg1 is the new window, arg2 is the new increment, arg3 is the new ambulation mode. This is used to update the values used to calculate regression features. 
  void updateRegVals() {
    char* args[3];
    uint16_t val1, val2, val3;  
    uint8_t index = 0; 
    do {
      args[index] = sCmd.next(); 
      index++; 
    } while (index < 3 && args[index] != NULL); 
    if (index == 3 && args[2] != NULL) {
      val1 = (uint16_t)atoi(args[0]); 
      val2 = (uint16_t)atoi(args[1]); 
      val3 = (uint8_t)atoi(args[2]);
      

      //let any extractions finish before regression settings
      chMtxLock(&Features::extractMtx);
      Features::regWin = val1; //window
      Features::regInc = val2; //increment
      Features::regIdx = val3;
      chMtxUnlock(&Features::extractMtx); 
    }
  }
  
  //This handles a classification query. When this function is called, the teensy will send a a packet containing features for classification. 
  //Takes 2 args. The first arg specifies the window size and the second is the gait location.
  void getClassificationFeatures() {
    //Send features packet
    char* arg = sCmd.next();
    uint8_t idx;
    uint16_t win;
    if (arg != NULL) {
      win = (uint16_t)atoi(arg);
      
      arg = sCmd.next();
      if(arg != NULL) {
        idx = (uint8_t)atoi(arg);
        
        chMtxLock(&Features::extractMtx);
        
        Features::classIdx = idx; //set gait location index
        
        classifQuery = true;
        Features::extractHelper(win, 1);//extract features for specified window size, type 1 (classification)
        classifQuery = false;
        
        chMtxUnlock(&Features::extractMtx);
      }
    }
  }
  
  //Sets a single mask at a time.
  //takes 4 args. the first 3 args become j, k, and l in mask[j][k][l]. The final arg should be a FEATS_NUM long character string representation of the feature mask
  //example: "F_MASK 0 1 3 100000000" sets the regression mask for channel 1, ambulation mode 3 to "true false false false false false false false false"
  //NOTE: there are more gait locations than ambulation modes, so some ambulation modes may not be set. This is ok
  void changeMask() {
    char* args[4]; 
    uint8_t j, k, l; 
    uint8_t index = 0; 
    do {
      args[index] = sCmd.next(); 
      index++; 
    } while (index < 4 && args[index] != NULL);
    if (args[3] != NULL && strlen(args[3]) == FEATS_NUM) {
      j = atoi(args[0]);
      k = atoi(args[1]);
      l = atoi(args[2]); 
      for (uint8_t i = 0; i < FEATS_NUM; i++) {  
        Features::mask[j][k][l][i] = (bool) (args[3][i] - '0'); 
      }
    } else {
      eriscommon::printText("Error: wrong number of arguments or mask was the wrong size.");
    }
  }

 void test(){
  Packet testpacket;
  testpacket.start(Packet::PacketType::TEXT);
  testpacket.append((uint8_t*)"hola James\n",12);
  testpacket.send();
 }

void INFO() {
  char temp[56];
  sprintf(temp, "Firmware: %s", FIRMWARE_INFO);
  eriscommon::printText(temp);
}

void StartThreads(){
  eriscommon::printText("Starting threads manually");
  KillThreads(); 
}

void StreamingStart(){
  stream_en = true;
  
  t0 = micros();//reset start time
}

void StreamingStop(){
  stream_en = false;
  eriscommon::printText("Streaming off");
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

//takes 1 arg. It specifies the index of the feature we want to print
void TransmitFEAT(){
  // Show number of missed points
  char* arg; 
  arg = sCmd.next(); 
  if (arg != NULL) {
    uint8_t ind = (uint8_t) atoi(arg); 
    if (ind >= 0 && ind < FEATS_NUM) {
      Serial.print("Features:");      
      // Show the data
      uint8_t i=0;
      for (i=0;i<BIOM_NUMCHANNELS;i++){
        Serial.print(Features::lastFeatures[i][ind],2);
        if (i != BIOM_NUMCHANNELS - 1) {
          Serial.print(",");
        }
      }
      Serial.println(); 
    }
  }
}

void TransmitBiom(){    
  chSysLockFromISR();
  BiomSample_t *samples=&biomsamples[0];
  int num=Biom::buffer.FetchData(samples,(char*)"BIOM",MEMBUFFERSIZE);
  long missed=Biom::buffer.missed();   
  chSysUnlockFromISR();   
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
