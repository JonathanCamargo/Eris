#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types that are general to the application and used in both Eris and external hardware
*/

typedef struct BiomSample{
  float timestamp;
  float ch[BIOM_NUMCHANNELS];    
} BiomSample_t;

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

typedef struct  __attribute((__packed__)) uint8_tSample{
  float timestamp;
  uint8_t value;    
} uint8_tSample_t;

typedef struct boolSample{
  float timestamp;
  bool value;    
} boolSample_t;

#endif
