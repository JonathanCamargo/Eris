#include "Eris.h"

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
String fsrFileName;
static File fsrFile; 
String sineFileName;
static File sineFile;
String syncFileName;
static File syncFile;

ErisBuffer<FSRSample_t> fsrbuffer;
ErisBuffer<floatSample_t> sinebuffer;
ErisBuffer<uint8_tSample_t> syncbuffer;

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
    
    RecordSamples<FSRSample_t>(fsrbuffer,fsrFile);
    RecordSamples<floatSample_t>(sinebuffer,sineFile);
    RecordSamples<uint8_tSample_t>(syncbuffer,syncFile);        
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    
    //digitalWrite(PIN_LED_R,!digitalRead(PIN_LED_R));
    chBSemSignal(&xwriteDataSemaphore);         
    chThdSleepMilliseconds(50);   
  }
}

bool CreateFiles(void){      
   // Create the files for sensors in subfolders for each sensor
   // Sine file
   sineFileName = String("sine/") + String(trialname) + String(".bin");    
   fsrFileName = String("fsr/") + String(trialname) + String(".bin");    
   syncFileName = String("sync/") + String(trialname) + String(".bin");    

   if (SD.exists(sineFileName.c_str())) {
    Serial.println("File exists... overwriting");
    SD.remove(sineFileName.c_str()); 
    SD.remove(fsrFileName.c_str());
    SD.remove(syncFileName.c_str());    
   }
  
   // FSR File
   SD.mkdir("fsr");     
   fsrFile= SD.open(fsrFileName.c_str(), O_WRITE | O_CREAT); 
   if(!fsrFile){
    return false;
   }
   #if DEBUG
    Serial.println("fsr file created");
   #endif
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
   #if DEBUG
   Serial.println("sine file created");
   #endif
   sineFile.println("Header(ms),sync(units)");

   SD.mkdir("sync");
   syncFile = SD.open(syncFileName.c_str(), O_WRITE | O_CREAT); 
   if(!syncFile){
    return false;
   }
   #if DEBUG
   Serial.println("sync file created");
   #endif
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
  chSysLockFromISR();
  recording=false;  
  chBSemReset(&xwriteDataSemaphore,true);
  chSysUnlockFromISR();    
  sineFile.close();
  fsrFile.close(); 
  syncFile.close();       
}

bool StartRecording(void){
  chSysLockFromISR();
  // Check SD and create files
  if (!isSDOK){
    Error::RaiseError(MEMORY,(char *)F("SDCARD:isSDOK")); 
    recording=false;
    return recording;
  }
  bool filesOk=CreateFiles();
  if (!filesOk){           
    recording=false;
    Error::RaiseError(MEMORY,(char *)F("SDCARD:CREATEFILES"));
    return recording; 
  }    
  ResetTime();
  recording=true;     
  chBSemSignal(&xwriteDataSemaphore); 
  chSysUnlockFromISR();
  //unlock mutex to make the LED blink  
  return recording; 
}

void ResetTime(){  
  chSysLockFromISR();
  startTime = micros(); 
  
  fsrbuffer.clear();
  sinebuffer.clear();
  syncbuffer.clear();
  chSysUnlockFromISR();
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

    fsrbuffer.init();
    sinebuffer.init();
    syncbuffer.init();
    
    // create tasks at priority lowest priority
    writeFiles=chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T),NORMALPRIO, WriteFiles_T, NULL);
 }
}
