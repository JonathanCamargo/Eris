#ifndef BUFFER_H
#define BUFFER_H

#include <Arduino.h>
#include "eriscommon.h"
#include "error.h"
#include <ChRt.h>

/* A class to handle buffers using mail boxes with chibios */

template<typename DataType>
class ErisBuffer{

typedef struct {
  DataType data;
  bool inuse=0;
}buffer_t;


public:
  buffer_t bufferData[MEMBUFFERSIZE]; //Buffer for data content (unordered)
  msg_t buffer_P[MEMBUFFERSIZE]; //Buffer for pointers to data content (ordered)
  mailbox_t buffer; //mailbox in chibios  
  long droppedCounter=0; //Number of samples dropped
  bool ready=false; //Buffer is initialized and operational
  bool append(DataType data); // Add new data to the buffer

  ErisBuffer(){}
  
  long missed(); // Return the number of missed samples so far and clear out the counter
  void init(); // Initialize buffer (Do this after the chibi kernel is launched)

  uint8_t FetchData(DataType* dest,char * text,uint8_t numSamples); // Request nsamples of data in the buffer
  void clear(); // Clears the buffer removing all it's contents
 
};



template<typename DataType>
  bool ErisBuffer<DataType>::append(DataType data){
    chSysLockFromISR();
     if (chMBGetFreeCountI(&this->buffer)>0){      
      uint8_t index=0;
      for (uint8_t i=0;i<MEMBUFFERSIZE;i++){
        if (!bufferData[i].inuse){
             index=i;
             break;   
        }
      }      
      this->bufferData[index].data=data;    
      this->bufferData[index].inuse=true;
      msg_t msg=(uint32_t)&(this->bufferData[index]);       
      (void)chMBPostI(&(this->buffer),msg);
      chSysUnlockFromISR();      
      return true;
    }
    else{    
      //Remove the first message from the mailbox and increment missed counter
      msg_t msg;
      chMBFetchI(&(this->buffer),&msg);            
      this->droppedCounter=this->droppedCounter+1;
      (*(buffer_t*)msg).inuse=true;
      (*(buffer_t*)msg).data=data;
      chMBPostI(&(this->buffer),msg);        
      chSysUnlockFromISR();      
      return false;
    }  
  
  }
  

template<typename DataType>
  long ErisBuffer<DataType>::missed(){
    long missed=this->droppedCounter;
    this->droppedCounter=0;
    return missed;   
  }
  
  template<typename DataType>
  void ErisBuffer<DataType>::clear(){    
    chMBResetI(&this->buffer);
	for (uint8_t i=0;i<MEMBUFFERSIZE;i++){
        bufferData[i].inuse=false;        
    }
	this->droppedCounter=0;    
	chMBResumeX(&this->buffer);
  }


template<typename DataType>
  void ErisBuffer<DataType>::init(){
     chMBObjectInit (&(this->buffer), this->buffer_P, MEMBUFFERSIZE);
  }



template <typename DataType>
uint8_t ErisBuffer<DataType>::FetchData(DataType* dest,char text[],uint8_t numSamples){  
   chSysLockFromISR();
   uint8_t num=chMBGetUsedCountI(&(this->buffer));
   uint8_t tosend=numSamples;
   if (numSamples>num){
    tosend=num;
   }
   for (int i=0;i<tosend;i++){
      msg_t msg;
      if(chMBFetchI(&(this->buffer),&msg)!=MSG_OK){
        Error::RaiseError(MEMORY,text);
      }
      dest[i]=(*(buffer_t *)msg).data;
      (*(buffer_t *)msg).inuse=false;
    }
  chSysUnlockFromISR();      
    return num;
}


#endif
