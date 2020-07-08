#include "fastdualbuffer.h"
#include "Arduino.h"
#include <util/atomic.h>

/*
FastDualBuffer::FastDualBuffer(){
  wbuf=buf0;
  rbuf=buf1;
  lock=false;
  idx=0;
  this->_size=DEFAULTFASTDUALBUFFERSIZE;
}

FastDualBuffer::FastDualBuffer(int size){
  // Constructor with size of buffer. Important size <= FASTDUALBUFFERSIZE
  wbuf=buf0;
  rbuf=buf1;
  lock=false;
  idx=0;
  _size=size;  
}


uint8_t FastDualBuffer::add(float x){
  wbuf[this->idx]=x;
  this->idx=this->idx+1;
  if (this->idx>=this->_size){
    this->idx=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    float * tmp=rbuf;
    rbuf=wbuf;
    wbuf=tmp;
	}
    if (lock==true){
      return -1;
    }
    return 1;
  }
  return 0;
}

int FastDualBuffer::size(){
    return this->_size;
}
float * FastDualBuffer::read(){
  return this->rbuf;
}

void FastDualBuffer::clear(){
  this->lock=false;
  this->idx=0;
}*/
