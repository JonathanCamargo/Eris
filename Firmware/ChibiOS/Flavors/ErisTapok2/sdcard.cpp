#include "Eris.h"

#include "emg.h"
#include "sinewave.h"

#include "sdcard.h"

#include <SPI.h>
#include <fastdualbuffer.h>

namespace SDCard{
    
const int chipSelect = BUILTIN_SDCARD;    
thread_t *writeFiles = NULL;
static binary_semaphore_t xwriteDataSemaphore;
static bool recording=false;  
static bool isSDOK=false;
long startTime=0;
char * DEFAULT_TRIALNAME = (char*)"test";;
char * trialname=DEFAULT_TRIALNAME;
char newLine = '\n'; 


/******************** Custom code should go here ****************************/
// Create file names for each type of data to store
String emgFileName;
static File emgFile; 
String fsrFileName;
static File fsrFile; 
String sineFileName;
static File sineFile;
String syncFileName;
static File syncFile;

ErisBuffer<EMGSample_t> emgbuffer;
ErisBuffer<FSRSample_t> fsrbuffer;
ErisBuffer<floatSample_t> sinebuffer;
ErisBuffer<boolSample_t> syncbuffer;

//Thread for periodically writing on the sdcard
static THD_WORKING_AREA(waWriteFiles_T, 8192);
static THD_FUNCTION(WriteFiles_T, arg) {  
  while(true){    
        
    msg_t msg = chBSemWaitTimeout(&xwriteDataSemaphore, MS2ST(TIME_IMMEDIATE));
    if (msg == MSG_TIMEOUT) {  
      chThdSleepMilliseconds(10);            
      continue;
    }          
    // Recording for each file
    RecordSamples<EMGSample_t>(emgbuffer,emgFile);
    RecordSamples<FSRSample_t>(fsrbuffer,fsrFile);
    RecordSamples<floatSample_t>(sinebuffer,sineFile);
    RecordSamples<boolSample_t>(syncbuffer,syncFile);    
    digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    chBSemSignal(&xwriteDataSemaphore);         
    chThdSleepMilliseconds(50);   
  }
}

void addSine(floatSample_t sample){  
  sinebuffer.append(sample);  
}

void addEMG(EMGSample_t sample){
  emgbuffer.append(sample);
}

void addFSR(FSRSample_t sample){
  fsrbuffer.append(sample);
}

void addSync(boolSample_t sample){
  syncbuffer.append(sample);
}



bool CreateFiles(void){   
   // Create the files for sensors in subfolders for each sensor
   // Sine file
   sineFileName = String("sine/") + String(trialname) + String(".bin"); 
   emgFileName = String("emg/") + String(trialname) + String(".bin"); 
   fsrFileName = String("fsr/") + String(trialname) + String(".bin");    
   syncFileName = String("sync/") + String(trialname) + String(".bin");    

   if (SD.exists(emgFileName.c_str())) {
    Serial.println("File exists... overwriting");
    SD.remove(sineFileName.c_str()); 
    SD.remove(emgFileName.c_str());
    SD.remove(fsrFileName.c_str());
    SD.remove(syncFileName.c_str());
   }
   
   // EMG FileSDCard::StartRecording();  
   SD.mkdir("emg");     
   emgFile = SD.open(emgFileName.c_str(), O_WRITE | O_CREAT); 
   if(!emgFile){
    return false;
   }
   emgFile.print("Header(ms)");
   for (int i = 0; i < EMG_NUMCHANNELS; i++) {
    emgFile.printf(",chan%d(V)", i);
   }
   emgFile.println();
  
   // FSR File
   SD.mkdir("fsr");     
   fsrFile= SD.open(fsrFileName.c_str(), O_WRITE | O_CREAT); 
   if(!fsrFile){
    return false;
   }
   fsrFile.print("Header(s)");
   for (int i = 0; i < FSR_NUMCHANNELS; i++) {
    fsrFile.printf(",fsr%d(V)", i);
   }
   fsrFile.println();

   SD.mkdir("sine");
   sineFile = SD.open(sineFileName.c_str(), O_WRITE | O_CREAT); 
   if(!sineFile){
    return false;
   }
   sineFile.println("Header(ms),sync(units)");

   SD.mkdir("sync");
   syncFile = SD.open(syncFileName.c_str(), O_WRITE | O_CREAT); 
   if(!syncFile){
    return false;
   }
   syncFile.println("Header(ms),sync(units)");



   return true;
}

/*******************************************************************************/

bool initSD(void){
   if (!SD.begin(chipSelect)) {
      eriscommon::printText("initialization failed. Things to check:");
      eriscommon::printText("* is a card inserted?");
      eriscommon::printText("* is your wiring correct?");
      eriscommon::printText("* did you change the chipSelect pin to match your shield or module?"); 
      return false;
    } else {
     eriscommon::printText("SD card is present."); 
     isSDOK = true;
     return true;
    }    
}
void StopRecording(void){
  recording=false;       
  sineFile.close();
  emgFile.close(); 
  fsrFile.close(); 
  syncFile.close();     
  digitalWrite(PIN_LED,LOW);
  chBSemReset(&xwriteDataSemaphore,true);
}

bool StartRecording(void){
  emgbuffer.clear();
  fsrbuffer.clear();
  sinebuffer.clear();
  syncbuffer.clear();
  // Check SD and create files
  if (!isSDOK){
    Error::RaiseError(MEMORY,(char *)"SDCARD"); 
    return false;
  }
  if (CreateFiles()){     
    recording=true;
    startTime = micros();      
  }
  else{
    recording=false;
    Error::RaiseError(MEMORY,(char *)"SDCARD");
    return false; 
  }  
  Serial.println("REC ON");
  chBSemSignal(&xwriteDataSemaphore); 
  //unlock mutex to make the LED blink  
  return true; 
}

void setTrialName(char * newtrialname){
  trialname=newtrialname;
}

char* getTrialName() {
  return trialname; 
}

void start(void){   
    recording=false;
    chBSemObjectInit(&xwriteDataSemaphore,true);
       
    // Initialize SDCard  
    initSD();

    emgbuffer.init();
    fsrbuffer.init();
    sinebuffer.init();
    syncbuffer.init();
    
    // create tasks at priority lowest priority
    writeFiles=chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T),NORMALPRIO, WriteFiles_T, NULL);
 }
}
