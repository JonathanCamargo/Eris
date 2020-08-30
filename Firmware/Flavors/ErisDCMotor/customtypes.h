#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types that are general to the application and used in both Eris and external hardware
*/


typedef struct IMUSample{
  float timestamp;
  float ax;
  float ay;
  float az;  
  float wx;
  float wy;
  float wz;
} IMUSample_t;

typedef struct FSRSample{
  float timestamp;
  float ch[FSR_NUMCHANNELS];    
} FSRSample_t;

typedef struct PotentiometerSample{
  float timestamp;
  float ch[POT_NUMCHANNELS];    
} PotentiometerSample_t;

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

typedef struct  __attribute((__packed__)) uint8_tSample{
  float timestamp;
  uint8_t value;    
} uint8_tSample_t;


#endif
