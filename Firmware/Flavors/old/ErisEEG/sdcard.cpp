#include "Eris.h"
#include "sinewave.h"


#include "sdcard.h"

#include <SD.h>
#include <SPI.h>
#include <fastdualbuffer.h>

namespace SDCard{

const int chipSelect = BUILTIN_SDCARD;    
thread_t *writeFiles = NULL;
static binary_semaphore_t xbufferFullSemaphore;\
static bool recording=false;
static bool isSDOK=false;
static char strbuffer[128];
char * DEFAULT_TRIALNAME = (char*)"test";;
char * trialname=DEFAULT_TRIALNAME;
File dataFile;

/******************** Custom code should go here ****************************/
// Create file names for each type of data to store
String sineFileName;
String eegFileName;
String fsrFileName;

// Define types of buffer data to store in SDCard (each one should map to a file)
static enum BUFFERTYPE{
  SINE,
  EEG,
  FSR
}bufferType;

// Create the buffers to temporary store data before sending to SD card
// In this case I will use FastDualBuffer, but any storage can be used.

// Buffer for sine values
static FastDualBuffer sineDataBuffer(SDBUFFERSIZE);
// Buffer for EEG values -multichannel-
static FastDualBuffer * EEGDataBuffers[NUMEEGCHANNELS];
// Buffer for FSR values
static FastDualBuffer fsrDataBuffer(SDBUFFERSIZE);


// A thread to write on the SD card when a temporary buffer is full 
static THD_WORKING_AREA(waWriteFiles_T, 4096);
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
          dataFile= SD.open(sineFileName.c_str(), FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)eegFileName.c_str());
            continue;
          }
          //Save into SDCARD     
          int buffersize=sineDataBuffer.size();
          float * data=sineDataBuffer.read();     
          for (int i=0;i<buffersize;i++){            
            int num=sprintf(strbuffer,"%i,%1.4f\n",i,data[i]);
            dataFile.write(strbuffer,num);                                
          }          
          //Close File             
          dataFile.close();                         
      }
      break;      
      case EEG:
      {
          //Open File                   
          dataFile= SD.open(eegFileName.c_str(), FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)eegFileName.c_str());
            continue;
          }
          //Save into SDCARD     
          int buffersize=EEGDataBuffers[0]->size();
          float * data[NUMEEGCHANNELS];
          for (uint8_t eegChan=0;eegChan<NUMEEGCHANNELS;eegChan++){
            data[eegChan]=EEGDataBuffers[eegChan]->read();
          }          
          for (int i=0;i<buffersize;i++){                       
            int num=sprintf(&strbuffer[0],"%i,%1.4f",i,data[0][i]);            
            for (uint8_t eegChan=1;eegChan<NUMEEGCHANNELS;eegChan++){
              num=num+sprintf(&strbuffer[num],",%1.4f",data[eegChan][i]);              
            } 
            num=num+sprintf(&strbuffer[num],"\n");                 
            dataFile.write(strbuffer,num);                           
          }          
          //Close File             
          dataFile.close();                         
      }
      break;
      case FSR:
      {
          //Open File                   
          dataFile= SD.open(fsrFileName.c_str(), FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)fsrFileName.c_str());
            continue;
          }
          //Save into SDCARD     
          int buffersize=fsrDataBuffer.size();
          float * data=fsrDataBuffer.read();     
          for (int i=0;i<buffersize;i++){            
            int num=sprintf(strbuffer,"%i,%1.4f\n",i,data[i]);
            dataFile.write(strbuffer,num);                                
          }          
          //Close File             
          dataFile.close();                         
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
     chBSemSignalI(&xbufferFullSemaphore);     
  }       
}

void addEEG(float value,uint8_t chan){ 
  if (!recording){return;}     
  if(chan==NUMEEGCHANNELS-1){
    if (EEGDataBuffers[chan]->add(value)){
     bufferType=EEG;                  
     chBSemSignalI(&xbufferFullSemaphore);     
    }           
  }
  else{
    EEGDataBuffers[chan]->add(value);
  }
}

void addFSR(float value){    
  if (!recording){return;}  
  if(fsrDataBuffer.add(value)){
     bufferType=FSR;                  
     chBSemSignalI(&xbufferFullSemaphore);     
  }       
}


bool CreateFiles(void){   
   //Sine data
   SD.mkdir("sine");
   sineFileName=String("sine/")+String(trialname)+String(".csv");
   dataFile= SD.open(sineFileName.c_str(), FILE_WRITE); 
   if(!dataFile){
    return false;
   }
   dataFile.println("sample,sine(units)");
   dataFile.close();    

   //EEG data
   SD.mkdir("eeg");
   eegFileName=String("eeg/")+String(trialname)+String(".csv");
   dataFile= SD.open(eegFileName.c_str(), FILE_WRITE); 
   if(!dataFile){
    return false;
   }
   dataFile.print("sample");
   for (uint8_t chan=0;chan<NUMEEGCHANNELS;chan++){
      dataFile.printf(",chan%d(mV)",chan);
   }
   dataFile.println();
   dataFile.close();    

   SD.mkdir("fsr");
   fsrFileName=String("fsr/")+String(trialname)+String(".csv");
   dataFile= SD.open(fsrFileName.c_str(), FILE_WRITE); 
   if(!dataFile){
    return false;
   }
   dataFile.println("sample,fsr(V)");
   dataFile.close();    
   
   return true;
}

/*******************************************************************************/

void StopRecording(void){
  recording=false;     
  dataFile.close();
}

void StartRecording(void){
  //Clear the buffers from any residual data from previous recordings
  sineDataBuffer.clear();
  if (!isSDOK){
    Error::RaiseError(MEMORY,(char *)"SDCARD");
    return;
  }
  if (CreateFiles()){     
    recording=true;
  }
  else{
    recording=false;
    Error::RaiseError(MEMORY,(char *)"SDCARD");
  }  
  Serial.println("REC ON");
}

void setTrialName(char * newtrialname){
  trialname=newtrialname;
}

void start(void){   
    recording=false;
    chBSemObjectInit(&xbufferFullSemaphore,true);
    //chCondObjectInit(&recordingCond);
    // Initialize SDCard  
    if (!SD.begin(chipSelect)) {
      Serial.println("initialization failed. Things to check:");
      Serial.println("* is a card inserted?");
      Serial.println("* is your wiring correct?");
      Serial.println("* did you change the chipSelect pin to match your shield or module?"); 
    } else {
     Serial.println("SD card is present."); 
     isSDOK=true;
    }    

    for (uint8_t eegChan=0;eegChan<NUMEEGCHANNELS;eegChan++){
       EEGDataBuffers[eegChan]=new FastDualBuffer(SDBUFFERSIZE);
    }
          
		// create tasks at priority lowest priority
    writeFiles=chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T),NORMALPRIO, WriteFiles_T, NULL);
    
		
	}
}
