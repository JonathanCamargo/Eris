#ifndef FASTDUALBUFFER_H
#define FASTDUALBUFFER_H

#define DEFAULTFASTDUALBUFFERSIZE 1024

#include <Arduino.h>
#include <util/atomic.h>

template <typename T=float,int bufferSize=1024> 
class FastDualBuffer{
   private:
	T buf0[bufferSize];
	T buf1[bufferSize];
	long idx; // Index of current written data
   	T *wbuf;//Buffer to write
   	T *rbuf;//Buffer to write
 	bool lock;
	int _size; //Size of the buffer (Defaults to FASTDUALBUFFERSIZE if other value is not supplied in constructor)

   public:
	
    	FastDualBuffer(){ // FastDualBuffer Constructor
		 wbuf=buf0;
		 rbuf=buf1;
		 lock=false;
		 idx=0;
		 this->_size=bufferSize;
	};

	uint8_t add(T x){ // Add a new element to the buffer returns true when buffer is full
		  wbuf[this->idx]=x;
		  this->idx=this->idx+1;
		  if (this->idx>=this->_size){
		    this->idx=0;
		    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			    T * tmp=rbuf;
			    rbuf=wbuf;
			    wbuf=tmp;
			    }
			    if (lock==true){
			      return -1;
			    }
			    return 1;
         	  }
		  return 0;
	};

	T *read(){ // Read access to the buffer
		return this->rbuf;
	};

	void clear(){
		 this->lock=false;
	  	this->idx=0;
	};


	int size(void){
		return bufferSize;
		};
};


#endif
