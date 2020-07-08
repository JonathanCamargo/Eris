#include "Eris.h"
#include "emg.h"
#include "sinewave.h"

#include <fastdualbuffer.h>
#include "sdcard.h"

#include <SD.h>
#include <SPI.h>

namespace SDCard{

static enum BUFFERTYPE{
  SINE,
  EMG
}bufferType;

File dataFile;
char emgFile[16]; // Emg file name

const int chipSelect = BUILTIN_SDCARD;
	
thread_t *writeFiles = NULL;

static binary_semaphore_t xbufferFullSemaphore;
time_measurement_t t;

volatile bool recording=false;
long startTime;
 
//Buffer for sine values
FastDualBuffer sineData;
static int sine_idx=0;

int fastdualbufferSize=FastDualBuffer::size();

//Buffer for EMG values
FastDualBuffer *emgData[NUMEMGCHANNELS];
FastDualBuffer emgTimeStampData;
static int emg_idx=0;


char  DEFAULT_FILENAME[10] = "test";
char  filename[10];
 
static THD_WORKING_AREA(waWriteFiles_T, 1024);
static THD_FUNCTION(WriteFiles_T, arg) {  
  while(true){    
    msg_t msg = chBSemWaitTimeout(&xbufferFullSemaphore, MS2ST(10));
    if (msg == MSG_TIMEOUT) {
      continue;
    }
    chTMStartMeasurementX(&t); 
          
    switch(bufferType){
    case SINE:
    {
        //Open File                   
        dataFile= SD.open("sine.txt", FILE_WRITE);         
        if(!dataFile){
          Error::RaiseError(MEMORY,(char *)"sine.csv");
          continue;
        }
        //Save into SDCARD          
        float *sinedata=sineData.read();      
        for (int i=0;i<fastdualbufferSize;i++){                                   
          int num=sprintf(strbuffer,"%i\t%1.4f\n",i,sinedata[i]);
          dataFile.write(strbuffer,num);
        }          
        //Close File             
        dataFile.close();                         
    }
    break;      
    case EMG:
    {
        //Open File                   
        dataFile= SD.open(emgFile, FILE_WRITE);         
        if(!dataFile){
          Error::RaiseError(MEMORY,(char *)emgFile);
          continue;
        }
        
        //Save into SDCARD
       int num;
       float *emgTimeStamp=emgTimeStampData.read(); 
       for (int i=0;i<fastdualbufferSize;i++){
           num=sprintf(strbuffer,"%1.5f",(double)emgTimeStamp[i]);
           for (int chan = 0; chan < NUMEMGCHANNELS; chan++) {
              float *emgdata=emgData[chan]->read();              
              num += sprintf(&strbuffer[num],",%1.5f",(double)emgdata[i]);                            
           }
           num += sprintf(&strbuffer[num],"\n");
           dataFile.write(strbuffer,num);    
           //Serial.println(strbuffer);     
        }                         
        //Close File   
        dataFile.close();  
        
      }
      break;  
      default:      
      break;
    }
    chTMStopMeasurementX(&t); 
    
  }   
}

void addSine(float value){
  if (!recording){
      return;
    }      
  if(sineData.add(value)){      
      bufferType = SINE;
      chBSemSignalI(&xbufferFullSemaphore);         
  }
  sine_idx=sine_idx+1;
}

void addEMG(float value, float currTime, uint8_t chan){
  if (!recording){
      return;
  }
     
  if(chan == 0) {
    emgTimeStampData.add(currTime);
  }

  if(chan == (NUMEMGCHANNELS - 1)) {
    if(emgData[chan]->add(value)){      
      bufferType = EMG;
      chBSemSignalI(&xbufferFullSemaphore);         
    } 
    emg_idx++;
  }
  else if (chan<(NUMEMGCHANNELS-1)){
    emgData[chan]->add(value);
  } 
}

bool initSD(void){
   if (!SD.begin(chipSelect)) {
      Serial.println("initialization failed. Things to check:");
      Serial.println("* is a card inserted?");
      Serial.println("* is your wiring correct?");
      Serial.println("* did you change the chipSelect pin to match your shield or module?"); 
      return false;
    } else {
     Serial.println("SD card is present."); 
     return true;
    }    
}

bool CreateFiles(void){   
   dataFile= SD.open("sine.txt", FILE_WRITE); 
   if(!dataFile){
    return false;
   }
   dataFile.println("sample\tsine(units)");
   dataFile.close(); 
  
   emgFile[0]='\0';
   strcat(emgFile,filename);
   strcat(emgFile,"emg.txt");
   
   Serial.println(emgFile);
   
   dataFile= SD.open(emgFile, FILE_WRITE); 
   
   if(!dataFile){
    return false;
   }
   dataFile.print("Time(ms)");
   
   for (int i = 0; i < NUMEMGCHANNELS; i++) {
    dataFile.printf(",chan%d(mV)", i);
   }
   
   dataFile.println();
   dataFile.close();
   
   return true;
}

void StopRecording(void){
  recording=false;
  for (int i=0;i<NUMEMGCHANNELS;i++){
      emgData[i]->clear();
    }
}

void StartRecording(void){    
  
  if (CreateFiles()){   
    startTime = micros();  
    recording=true;
  }
  else{
    if (!initSD()){
      recording=false;
      Error::RaiseError(MEMORY,(char *)"SDCARD");
      return;
    }
    else{
      recording=false;
    }
  }
  Serial.println("REC ON");
}

void start(void){   
    recording=false;
    chBSemObjectInit(&xbufferFullSemaphore,true);
    chTMObjectInit(&t);
    
    for (int i=0;i<NUMEMGCHANNELS;i++){
      emgData[i]=new FastDualBuffer();
    }

   
    // Initialize SDCard  
    initSD();
    
		// create tasks at priority lowest priority
    writeFiles=chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T),NORMALPRIO, WriteFiles_T, NULL);
	}
}
