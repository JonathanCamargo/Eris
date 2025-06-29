#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types that are general to the application and used in both Eris and external hardware
*/

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

typedef struct multiSample{
  float timestamp;
  float value[NUMCHANNELS];    
} multiSample_t;

typedef struct  __attribute((__packed__)) uint8_tSample{
  float timestamp;
  uint8_t value;    
} uint8_tSample_t;


#endif
