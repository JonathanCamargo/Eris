#include "Eris.h"
#include "emg.h"
#include "sinewave.h"

#include "sdcard.h"

#include <SD.h>
#include <SPI.h>
#include <fastdualbuffer.h>

namespace SDCard{
    
const int chipSelect = BUILTIN_SDCARD;    
thread_t *writeFiles = NULL;
static binary_semaphore_t xbufferFullSemaphore;
static bool recording=false;  
static bool isSDOK=false;
static uint8_t lastEtiChan=-1; 
static char strbuffer[128];
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

File* currFile; 

// Define types of buffer data to store in SDCard (each one should map to a file)
static enum BUFFERTYPE{
  SINE,
  EMG ,
  FSR 
}bufferType;

// Create the buffers to temporary store data before sending to SD card
// In this case I will use FastDualBuffer, but any storage can be used.

// Buffer for sine values
static FastDualBuffer<float,SDBUFFERSIZE> sineDataBuffer;
// Buffer for EMG values -multichannel-
static FastDualBuffer<float,SDBUFFERSIZE> * EMGDataBuffers[NUMEMGCHANNELS];
// Buffer for FSR values
static FastDualBuffer<float,SDBUFFERSIZE_FSR> * FSRDataBuffers [NUMFSRCHANNELS];

// Buffers for timestamps
static FastDualBuffer<float,SDBUFFERSIZE> EMGTimeBuffer;
static FastDualBuffer<float,SDBUFFERSIZE> FSRTimeBuffer;

static long temp = 0;

static THD_WORKING_AREA(waWriteFiles_T, 8192);
static THD_FUNCTION(WriteFiles_T, arg) {  
  while(true){    
    msg_t msg = chBSemWaitTimeout(&xbufferFullSemaphore, MS2ST(10));
    if (msg == MSG_TIMEOUT) {
      continue;
    }  
    if (!recording){
      continue;
    }  
    switch(bufferType){
    case SINE:
    {
        //Open File                   
        currFile = &sineFile;         
        if(! *currFile){
          Error::RaiseError(MEMORY,(char *)"sine.csv");
          continue;
        }
        //Save into SDCARD     
        int buffersize=sineDataBuffer.size();
        float * data=sineDataBuffer.read();  
        for (uint8_t i=0;i<buffersize;i++){          
          memcpy(&strbuffer[0], &data[i], sizeof(float));   
          currFile->write(strbuffer, sizeof(float) + 1);   
        }          
        //Close File             
        currFile->flush();                                
    }
     case EMG:
    {
        //Open File                   
          currFile = &emgFile;         
          if(!currFile){
            Error::RaiseError(MEMORY,(char *)emgFileName.c_str());
            continue;
          }
          //Save into SDCARD     
          int buffersize=EMGDataBuffers[0]->size();
          float * data[NUMEMGCHANNELS];
          float * dataTime;
          dataTime=EMGTimeBuffer.read();
          for (uint8_t emgChan=0;emgChan<NUMEMGCHANNELS;emgChan++){
            data[emgChan]=EMGDataBuffers[emgChan]->read();
          }

          for (int i=0;i<buffersize;i++){                       
            int num=0;
            memcpy(&strbuffer[0], &dataTime[i], sizeof(float));
            num += sizeof(float);   
            for (uint8_t emgChan = 0; emgChan < NUMEMGCHANNELS; emgChan++){
              memcpy(&strbuffer[num], &data[emgChan][i], sizeof(float)); 
              num += sizeof(float);
            } 
            memcpy(&strbuffer[num], &newLine, sizeof(char)); 
            num++;                 
            currFile->write(strbuffer, num);                           
          }          
          //Close File             
          currFile->flush();
          //Serial.println(((float)(micros() - temp))/1000.0);                                            
      }
      break;  
       case FSR:
    {
      
       //Open File                   
          currFile = &fsrFile;         
          if(! *currFile){
            Error::RaiseError(MEMORY,(char *)fsrFileName.c_str());
            continue;
          }
          //Save into SDCARD     
          int buffersize=FSRDataBuffers[0]->size();
          float * data[NUMFSRCHANNELS];
          float * dataTime;
          dataTime=FSRTimeBuffer.read();
          for (uint8_t fsrChan=0;fsrChan<NUMFSRCHANNELS;fsrChan++){
            data[fsrChan]=FSRDataBuffers[fsrChan]->read();
          }          
          for (int i=0;i<buffersize;i++){                       
            int num=0;    
            memcpy(&strbuffer[0], &dataTime[i], sizeof(float));  
            num += sizeof(float);  
            for (uint8_t fsrChan=0;fsrChan<NUMFSRCHANNELS;fsrChan++){
              memcpy(&strbuffer[num], &data[fsrChan][i], sizeof(float));
              num += sizeof(float);             
            } 
            memcpy(&strbuffer[3*NUMFSRCHANNELS*sizeof(float)], &newLine, sizeof(char)); 
            num++;      
            currFile->write(strbuffer,num);    
          }          
          //Close File             
          currFile->flush();         
                              
      }
      break;  
      default:      
      break;
    }
  }   
}

void addSine(float value){
  if (!recording){return;}
  if(sineDataBuffer.add(value)){
     bufferType=SINE;                  
     //chBSemSignalI(&xbufferFullSemaphore);     
  }       
}

void addEMG(float value, float currTime, uint8_t chan){
  if (!recording){return;}  
  if (chan == 0){
    EMGTimeBuffer.add(currTime);
  }
      
  if(chan == (NUMEMGCHANNELS - 1)) {
    if(EMGDataBuffers[chan]->add(value)){      
      bufferType = EMG;
      
      temp = micros();
      chBSemSignalI(&xbufferFullSemaphore);         
    }
  } else if (chan < NUMEMGCHANNELS){
    EMGDataBuffers[chan]->add(value);
  }
}

void addFSR(float value, float currTime, uint8_t chan){
  if (!recording){return;}  
  if (chan==0){
    FSRTimeBuffer.add(currTime);
  } 
  if(chan == (NUMFSRCHANNELS - 1)) {
    if(FSRDataBuffers[chan]->add(value)){      
      bufferType = FSR;
      
      chBSemSignalI(&xbufferFullSemaphore);         
    }     
  }
  else if (chan<NUMFSRCHANNELS){
    FSRDataBuffers[chan]->add(value);
  }
}


bool CreateFiles(void){   
   // Create the files for sensors in subfolders for each sensor
   // Sine file
   sineFileName = String("sine/") + String(trialname) + String(".bin"); 
   emgFileName = String("emg/") + String(trialname) + String(".bin"); 
   fsrFileName = String("fsr/") + String(trialname) + String(".bin");    

   if (SD.exists(emgFileName.c_str())) {
    Serial.println("File exists... overwriting");
    SD.remove(sineFileName.c_str()); 
    SD.remove(emgFileName.c_str());
    SD.remove(fsrFileName.c_str());
   }
   
   SD.mkdir("sine");
   sineFile = SD.open(sineFileName.c_str(), O_WRITE | O_CREAT); 
   if(!sineFile){
    return false;
   }
   sineFile.println("sample,sine(units)");

   // EMG FileSDCard::StartRecording();  
   SD.mkdir("emg");     
   emgFile = SD.open(emgFileName.c_str(), O_WRITE | O_CREAT); 
   if(!emgFile){
    return false;
   }
   emgFile.print("time(s)");
   for (int i = 0; i < NUMEMGCHANNELS; i++) {
    emgFile.printf(",chan%d(V)", i);
   }
   emgFile.println();
  
   // FSR File
   SD.mkdir("fsr");     
   fsrFile= SD.open(fsrFileName.c_str(), O_WRITE | O_CREAT); 
   if(!fsrFile){
    return false;
   }
   fsrFile.print("time(s)");
   for (int i = 0; i < NUMFSRCHANNELS; i++) {
    fsrFile.printf(",fsr%d(V)", i);
   }
   fsrFile.println();

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
}

bool StartRecording(void){
  //Clear the buffers from any residual data from previous recordings
  sineDataBuffer.clear(); 
  EMGTimeBuffer.clear(); 
  FSRTimeBuffer.clear(); 
   
  for (uint8_t emgChan=0;emgChan<NUMEMGCHANNELS;emgChan++){
     EMGDataBuffers[emgChan]->clear();
  }
  for (uint8_t fsrChan=0; fsrChan<NUMFSRCHANNELS; fsrChan++) {
    FSRDataBuffers[fsrChan]->clear(); 
  } 

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
  //Serial.println("REC ON");
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
    chBSemObjectInit(&xbufferFullSemaphore,true);
    //chCondObjectInit(&recordingCond);
    // Initialize SDCard  
    initSD();

    for (uint8_t emgChan=0;emgChan<NUMEMGCHANNELS;emgChan++){
       EMGDataBuffers[emgChan]=new FastDualBuffer<float,SDBUFFERSIZE>;
    }
    
    for (uint8_t fsrChan=0;fsrChan<NUMFSRCHANNELS;fsrChan++){
       FSRDataBuffers[fsrChan]=new FastDualBuffer<float,SDBUFFERSIZE_FSR>;
    } 
          
    // create tasks at priority lowest priority
    writeFiles=chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T),NORMALPRIO, WriteFiles_T, NULL);
 }
}
