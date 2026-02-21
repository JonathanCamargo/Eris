#include "Eris.h"
#include "loadcell.h"

#include <sriloadcell.h>

#if defined(__arm__) && defined(CORE_TEENSY)  
#endif
#if defined(__SAM3X8E__) || defined(__SAM3X8H__)
  #include <due_can.h>
#endif


namespace Loadcell{

  //Buffer for readings
  static msg_t buffer_P[BUFFERSIZE];
  static buffer_t bufferData[BUFFERSIZE]; 
  mailbox_t buffer;
 
  long missed; 

  bool firstTime=true;
  
  //Elmo driver connected to CANID
  static SRILoadcell loadcell(SRILOADCELL_CANID);
  
  //Scheduled tasks  
 
  thread_t *readDatal = NULL;// TODO: ReadCurrent_T (100Hz) read current in motor

void UpdateState(LoadcellDataRaw_t newState){
  //Receive a new load cell state object and upload to the buffer queue
  
  chSysLockFromISR();
  //Check if we have free space in the mailbox    
  if (chMBGetFreeCountI(&buffer)>0){      
    uint8_t index=0;
    for (uint8_t i=0;i<BUFFERSIZE;i++){
      if (!bufferData[i].inuse){
           index=i;
           break;   
      }
    }
    //copy newState to the data in buffer
    memcpy(&bufferData[index].data,&newState,sizeof(LoadcellDataRaw_t));
    bufferData[index].inuse=true;
    msg_t msg=(uint32_t)&bufferData[index];       
    (void)chMBPostI(&buffer,msg);
  }
  else{    
    //Remove the first message from the mailbox and increment missed counter
    msg_t msg;
    chMBFetchI(&buffer,&msg);            
    missed=missed+1;
    (*(buffer_t*)msg).inuse=true;
    memcpy(&((*(buffer_t*)msg).data),&newState,sizeof(LoadcellDataRaw_t));
    chMBPostI(&buffer,msg);        
  }
  chSysUnlockFromISR();   
}

  
static THD_WORKING_AREA(waReadData_T, 512);
static THD_FUNCTION(ReadData_T, arg) {  
  // Read the loadcell
    while (!chThdShouldTerminateX()){      
      bool readingsOk=true;
      LoadcellDataRaw_t rawdata;
      readingsOk=loadcell.get_rawdata(&rawdata);
      
      // Get readings in SRI library struct and then u
      //LoadcellData_t data=frame2data((uint8_t *)&rawdata);

      if (readingsOk){ 
        UpdateState((LoadcellDataRaw_t)rawdata);
      }
      else{
        Error::RaiseError(CANBUS,(char *)"LOADCELL");
      }
      chThdSleepMicroseconds(SRILOADCELL_RATE_US);
   }
   Serial.println("Terminating Loadcell Thread");
 }



void start(void){  
    missed=0;   
    
    if (firstTime){
      chMBObjectInit (&buffer, buffer_P, BUFFERSIZE);
      firstTime=false;
    }
    // **************Hardware selection ************ //
    // Start CAN
    #if defined(__MK20DX256__) && defined(CORE_TEENSY)  //Teensy 3.2
    Can0.begin(1000000);
    #endif    
    
    #if defined(__MK66FX1M0__) && defined(CORE_TEENSY)  //Teensy 3.6
    Can1.begin(1000000);
    #endif    
    #if defined(__SAM3X8E__) || defined(__SAM3X8H__) //Arduino Due
    Can1.begin(1000000);    
    #endif
    // *********************************************** //
    
    //bool loadcellOk=false;
    bool loadcellOk=false;
    if (loadcell.connect()){
      Serial.println("Connecting to loadcell");
      LoadcellDataRaw_t rawdata;
      loadcellOk=loadcell.get_rawdata(&rawdata);      
    }
    else{
      loadcellOk=false;
      Error::RaiseError(CANBUS);       
    }
    
    if (loadcellOk){            
       readDatal=chThdCreateStatic(waReadData_T, sizeof(waReadData_T),NORMALPRIO, ReadData_T, NULL);
	  }	
    else{       
       Error::RaiseError(CANBUS,(char *)"loadcell");       
    }
}

void kill(){
  if (readDatal!=NULL){
    chThdTerminate(readDatal);
  }
}
}
