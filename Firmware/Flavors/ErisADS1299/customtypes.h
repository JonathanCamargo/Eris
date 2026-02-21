#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types that are general to the application and used in both Eris and external hardware
*/

typedef struct EMGSample{
  float timestamp;
  float ch[EMG_NUMCHANNELS];    
} EMGSample_t;

typedef struct FSRSample{
  float timestamp;
  float ch[FSR_NUMCHANNELS];    
} FSRSample_t;

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

typedef struct  __attribute((__packed__)) uint8_tSample{
  float timestamp;
  uint8_t value;    
} uint8_tSample_t;


#endif
