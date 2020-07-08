#include "fastdualbuffer.h"
#include "Arduino.h"

FastDualBuffer::FastDualBuffer(){
  wbuf=buf0;
  rbuf=buf1;
  lock=false;
  idx=0;
}

int FastDualBuffer::add(float x){
  wbuf[this->idx]=x;
  this->idx=this->idx+1;
  if (this->idx>=FASTDUALBUFFERSIZE){
    this->idx=0;
    float * tmp=rbuf;
    rbuf=wbuf;
    wbuf=tmp;
    if (lock==true){
      return -1;
    }
    return 1;
  }
  return 0;
}

float * FastDualBuffer::read(){
  return this->rbuf;
}

void FastDualBuffer::clear(){
  this->lock=false;
}
