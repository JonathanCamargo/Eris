#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/*//////////////////////////////////////////// 

Definition of custom types for data exchange 

User can add more types as needed

////////////////////////////////////////////*/

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

typedef struct  __attribute((__packed__)) uint8_tSample{
  float timestamp;
  uint8_t value;    
} uint8_tSample_t;


#endif
