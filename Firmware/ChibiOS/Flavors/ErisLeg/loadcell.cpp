#include "Eris.h"
#include "loadcell.h"
#include "configuration.h"

#include <sriloadcell.h>

namespace Loadcell{

  //Buffer for readings
  ErisBuffer<LoadcellSample_t> buffer;
 
  bool firstTime=true;
  
  //Elmo driver connected to CANID
  static SRILoadcell loadcell(SRILOADCELL_CANID);
  
  //Scheduled tasks  
 
  thread_t *readDatal = NULL;// TODO: ReadCurrent_T (100Hz) read current in motor

  
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
	    // Transform the rawdata to regular data
		LoadcellData_t data=frame2data((uint8_t *)&rawdata);
		LoadcellSample_t sample;
		
		sample.timestamp=micros()-t0;
		sample.forceX=data.forceX;
		sample.forceY=data.forceY;
		sample.forceZ=data.forceZ;
		sample.momentX=data.momentX;
		sample.momentY=data.momentY;
		sample.momentZ=data.momentZ;
		
		buffer.append(sample);
      }
      else{
        Error::RaiseError(CANBUS,(char *)"LOADCELL");
      }
      chThdSleepMicroseconds(SRILOADCELL_RATE_US);
   }
   eriscommon::println("Terminating Loadcell Thread");
 }



void start(void){  
    
    if (firstTime){
      buffer.init();
      firstTime=false;
    }

    Can0.begin(1000000);

    // *********************************************** //
    
    bool loadcellOk=false;
    if (loadcell.connect()){
      eriscommon::println("Connecting to loadcell");
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
