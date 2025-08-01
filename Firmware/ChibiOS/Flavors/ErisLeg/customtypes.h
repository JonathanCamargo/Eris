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

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

typedef struct ImpedanceParameters{
  float timestamp;
  float k;
  float b;
  float theta_eq;    
} ImpedanceParameters_t;

typedef struct JointStateSample{
  float timestamp;
  float theta;
  float theta_dot;
  float torque;    
} JointStateSample_t;

typedef struct LoadcellSample{
  float timestamp;
  float forceX;
  float forceY;
  float forceZ;
  float momentX;
  float momentY;
  float momentZ;   
} LoadcellSample_t;

typedef struct  __attribute((__packed__)) uint8_tSample{
  float timestamp;
  uint8_t value;    
} uint8_tSample_t;


#endif
