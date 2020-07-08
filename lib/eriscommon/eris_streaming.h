#ifndef ERIS_STREAMING_H

#include "eriscommon.h"


template <typename T,uint8_t numsamples>
uint8_t StreamSamples(ErisBuffer<T> &buffer, Packet &packet){
   /*
   * StreamSamples takes the information from a buffer and appends at most numsamples to the packet.
   *
   */
   T samples[numsamples];
   uint8_t num=buffer.FetchData(samples,(char *)"stream",numsamples);
   //This returned all the samples that are in the buffer which can be
   // more or less than requested

   uint8_t toSend=(num<numsamples ? num : numsamples);   
   
   //long missed=buffer.missed();                 
   //Transmit buffer

   //Send the number of samples first as an uint8_t number
   
   packet.append((uint8_t *)&toSend,sizeof(uint8_t));

   //Add all the samples next to this
   for (uint8_t i=0;i<toSend;i++){
      packet.append((uint8_t *)&samples[i],sizeof(T));
   }     
   return toSend;
  }

template <typename T,uint8_t numsamples>
uint8_t StreamSamplesMemoryEfficient(ErisBuffer<T> &buffer, Packet &packet,T * samplesbuffer){
   /*
   * StreamSamples takes the information from a buffer and appends at most numsamples to the packet.
   *
   */
   //T samplesbuffer[numsamples];
   uint8_t num=buffer.FetchData(samplesbuffer,(char *)"stream",numsamples);
   //This returned all the samples that are in the buffer which can be
   // more or less than requested

   uint8_t toSend=(num<numsamples ? num : numsamples);   
   
   //long missed=buffer.missed();                 
   //Transmit buffer

   //Send the number of samples first as an uint8_t number
   
   packet.append((uint8_t *)&toSend,sizeof(uint8_t));

   //Add all the samples next to this
   for (uint8_t i=0;i<toSend;i++){
      packet.append((uint8_t *)&samplesbuffer[i],sizeof(T));
   }     
   return toSend;
  }



  
  #endif
