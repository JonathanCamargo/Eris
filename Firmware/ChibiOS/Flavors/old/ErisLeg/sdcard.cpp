#include "Eris.h"

#include "sinewave.h"
#include "joints.h"
#include "loadcell.h"

#include "sdcard.h"

#include <SD.h>
#include <SPI.h>

namespace SDCard{

static enum BUFFERTYPE{
  SINE,
  JOINT,
  LOADCELL 
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

//Buffer for joint values
static JointState_t jointData[SDBUFFERSIZE]; 
static JointState_t jointDataTmp[SDBUFFERSIZE]; 
static int joint_idx=0;


//Buffer for joint values
static LoadCellData_t LoadCellData[SDBUFFERSIZE]; 
static LoadCellData_t LoadCellDataTmp[SDBUFFERSIZE]; 
static int loadcell_idx=0;



 //Buffer for strings
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
          dataFile= SD.open("sine.csv", FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)"sine.csv");
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
      case JOINT:
      {
          //Open File                   
          dataFile= SD.open("joint.txt", FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)"joint.txt");
            continue;
          }
          //Save into SDCARD          
          for (int i=0;i<SDBUFFERSIZE;i++){            
            JointState_t a=jointDataTmp[i];
            int num=sprintf(buffer,"%i\t%1.4f\t%1.4f\t%1.4f\t%1.4f\t%1.4f\n",i,a.thetaKnee,a.thetadotKnee,a.torqueKnee,a.thetaAnkle,a.thetadotAnkle,a.torqueAnkle);
            dataFile.write(buffer,num);                                
          }          
          //Close File             
          dataFile.close();            
      }
      break;   
     case LOADCELL:
      {
          //Open File                   
          dataFile= SD.open("loadcell.txt", FILE_WRITE);         
          if(!dataFile){
            Error::RaiseError(MEMORY,(char *)"loadcell.txt");
            continue;
          }
          //Save into SDCARD          
          for (int i=0;i<SDBUFFERSIZE;i++){            
            LoadcellData_t a=LjointDataTmp[i];
            int num=sprintf(buffer,"%i\t%1.4f\t%1.4f\t%1.4f\t%1.4f\t%1.4f\t%1.4f\n",i,a.forceX,a.forceY,a.forceZ,a.momentX,a.momentY,a.momentZ);
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
  if (!recording){
      return;
    }      
  if(sine_idx>=SDBUFFERSIZE){
     //Copy buffer
     memcpy(sineDataTmp,sineData,sizeof(float)*SDBUFFERSIZE);
     sine_idx=0;
     bufferType=SINE;                  
     chBSemSignalI(&xbufferFullSemaphore);     
  }     
  sineData[sine_idx]=value;  
  sine_idx=sine_idx+1;
}


void addJoint(JointState_t value){
  joint_idx=joint_idx+1;
  if(joint_idx>=SDBUFFERSIZE){
     //Copy buffer
     memcpy(jointDataTmp,jointData,sizeof(JointState_t)*SDBUFFERSIZE);
     joint_idx=0;
     bufferType=JOINT;                  
     chBSemSignalI(&xbufferFullSemaphore);     
  }     
  jointData[joint_idx]=value;  
  
}


void addLoadcellData(LoadcellData_t value){
  loadcell_idx=loadcell_idx+1;
  if(loadcell_idx>=SDBUFFERSIZE){
     //Copy buffer
     memcpy(loadcellDataTmp,loadcellData,sizeof(LoadcellData_t)*SDBUFFERSIZE);
     loadcell_idx=0;
     bufferType=LOADCELL;                  
     chBSemSignalI(&xbufferFullSemaphore);     
  }     
  loadcellData[loadcell_idx]=value;    
}


bool CreateFiles(void){   
   dataFile= SD.open("sine.csv", FILE_WRITE); 
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

   dataFile= SD.open("loadcell.txt", FILE_WRITE);
   if(!dataFile){
    return false;
   }
   dataFile.println("sample\tForceX(N)\tForceY(N)\tForceZ(N)\tMomentX(Nm)\tMomentY(Nm)\tMomentZ(Nm)\n");
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
