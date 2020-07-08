#ifndef ERIS_SD_H

#include "eriscommon.h"
#include <SD.h>


template<typename T>
uint8_t RecordSamples(ErisBuffer<T> &buffer,File &file){    
        if(!file){
          Error::RaiseError(MEMORY,(char *)file.name());
          return 0;
        }       
        //Fetch data from buffer
        T samples[MEMBUFFERSIZE];
	chSysLockFromISR();
        uint8_t num=buffer.FetchData(samples,(char*)file.name(),MEMBUFFERSIZE);
        chSysUnlockFromISR();
	//long missed=buffer.missed();           
        //Save into SDCARD     
        for (uint8_t i=0;i<num;i++){                    
          file.write((uint8_t*)&samples[i], sizeof(T));
          file.println();
        }          
        //Close File             
        return num;                      
}


#endif
