#include "Eris.h"

#include "serialselector.h"


namespace SerialSelector{
  eris_thread_ref_treadSerial = NULL;  
  
  /********************** Threads *********************************/
	   
	ERIS_THREAD_WA(waReadSerial_T, 128);
	ERIS_THREAD_FUNC(ReadSerial_T) {  		  	 
	  while(1){		    
		//Check serial buffer and see if there is anything there 			
		ReadSerial();		   
		eris_sleep_ms(100);
	  }
	}

  /***************************************************************/

   bool ReadSerial(){   
    bool returnFlag=0;
     if (EMG_SERIAL.available()){
         //Do something?
         returnFlag=0;
     }
     return returnFlag;
   }
  
 void SelectNegativeElectrode(uint8_t negElectrodeNum){
    if (negElectrodeNum > 0 && negElectrodeNum <= 9) {       
      // Switch the port according to the commands
      EMG_SERIAL.write(0xAA);
      EMG_SERIAL.write(negElectrodeNum);
      EMG_SERIAL.write(0xBB);
    }
    else {
      #if DEBUG
      eriscommon::println("Not a valid electrode number.");      
      #endif
    }
  }

  
	void start(void){ 
   EMG_SERIAL.begin(115200);   
 
   while (!EMG_SERIAL) {
      ; // wait for //Serial port to connect.
   }

   eriscommon::println("Electrode selector ready");
  	// create task at priority one
	readSerial=eris_thread_create(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);    
	}

}
