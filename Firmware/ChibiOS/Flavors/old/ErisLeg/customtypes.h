#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types that are general to the application and used in both Eris and external hardware
*/


typedef struct{
  float kKnee;
  float bKnee;
  float theta_eqKnee;	
  float kAnkle;
  float bAnkle;
  float theta_eqAnkle;
} ImpedanceParameters_t;

typedef struct{
  float thetaKnee;
  float thetadotKnee;
  float torqueKnee; 
  float thetaAnkle;
  float thetadotAnkle;
  float torqueAnkle;
} JointState_t;

typedef struct{
  uint16_t thetaKnee;      // Range JOINT_MINPOS JOINT_MAXPOS
  uint16_t thetadotKnee;   // Range JOINT_MINVEL JOINT_MAXVEL
  uint16_t torqueKnee;     // Range JONIT_MINTOR JOINT_MAXTOR
  uint16_t thetaAnkle;     
  uint16_t thetadotAnkle;
  uint16_t torqueAnkle;
} JointStateRaw_t;

#ifdef ERISDRIVER_H   // #ifdef to only define this for Eris driver
// Defined in sriloadcell.h
 typedef struct LoadcellData {
  float forceX;
  float forceY;
  float forceZ;
  float momentX;
  float momentY;
  float momentZ;
} LoadcellData_t;

typedef struct LoadcellDataRaw {
  uint8_t loadcellvalues[8];
} LoadcellDataRaw_t;
#endif


#endif
