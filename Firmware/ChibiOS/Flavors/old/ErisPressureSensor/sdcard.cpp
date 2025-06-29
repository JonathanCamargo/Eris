#include "Eris.h"
#include "sinewave.h"


#include "sdcard.h"

#include <SD.h>
#include <SPI.h>

namespace SDCard{

static enum BUFFERTYPE{
  SINE
}bufferType;

File dataFile;

const int chipSelect = BUILTIN_SDCARD;    
	
thread_t *writeFiles = NULL;

static binary_semaphore_t xbufferFullSemaphore;

static int recording=false;
  
//Buffer for sine values
static float sineData[SDBUFFERSIZE]; 
static float sineDataTmp[SDBUFFERSIZE]; 
static int sine_idx=0;

static char buffer[128];
 
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
          dataFile= SD.open("sine.txt", FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)"sine.txt");
            continue;
          }
          //Save into SDCARD          
          for (int i=0;i<SDBUFFERSIZE;i++){            
            int num=sprintf(buffer,"%i\t%1.4f\n",i,sineDataTmp[i]);
            dataFile.write(buffer,num);                                
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
  sine_idx=sine_idx+1;
  if(sine_idx>=SDBUFFERSIZE){
     //Copy buffer
     memcpy(sineDataTmp,sineData,sizeof(float)*SDBUFFERSIZE);
     sine_idx=0;
     bufferType=SINE;                  
     chBSemSignalI(&xbufferFullSemaphore);     
  }     
  sineData[sine_idx]=value;  
}

bool CreateFiles(void){   
   dataFile= SD.open("sine.txt", FILE_WRITE); 
   if(!dataFile){
    return false;
   }
   dataFile.println("sample\tsine(units)\n");
   dataFile.close();
   
   dataFile= SD.open("joint.txt", FILE_WRITE);
   if(!dataFile){
    return false;
   }
   dataFile.println("sample\tThetaKnee(deg)\tThetadotKnee\tTorqueKnee(Nm)\tThetaAnkle(deg/s)\tThetadotAnkle(deg/s)\tTorqueAnkle(Nm)\n");
   dataFile.close();
   
   dataFile= SD.open("analog.txt", FILE_WRITE); 
   if(!dataFile){
    return false;
   }
   dataFile.println("sample\tanalog(V)\n");
   dataFile.close();
   return true;
}

void StopRecording(void){
  recording=false;    
  dataFile.close();
}

void StartRecording(void){  
  if (CreateFiles()){     
    recording=true;
  }
  else{
    recording=false;
    Error::RaiseError(MEMORY,(char *)"SDCARD");
  }
  Serial.println("REC ON");
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
    }    
     
		// create tasks at priority lowest priority
    writeFiles=chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T),NORMALPRIO, WriteFiles_T, NULL);
    
		
	}
}
