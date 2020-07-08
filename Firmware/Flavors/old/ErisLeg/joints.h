#ifndef JOINTS_H
#define JOINTS_H

#include <Arduino.h>
#include "customtypes.h"
#include "buffers.h"

namespace Joints{

extern ErisBuffer<JointState_t> buffer;

#if DEBUG_TIME
extern  time_measurement_t time;
#endif

void start(void);
void kill();

double ImpedanceLawKnee(JointState_t);
double ImpedanceLawAnkle(JointState_t);

void ReadIncrementalData();

void UpdateState(JointState_t);
void UpdateCurrent_T(void *arg);
void SetIP(float k_Knee,float b_Knee, float theta_eq_Knee,float k_Ankle,float b_Ankle, float theta_eq_Ankle);
void SetIPA(float k, float b, float theta);
void SetIPK(float k, float b, float theta);
void SetIP(ImpedanceParameters_t params);
void GetIP(float &k_Knee,float &b_Knee, float &theta_eq_Knee,float &k_Ankle,float &b_Ankle, float &theta_eq_Ankle);
void GetIP(ImpedanceParameters_t &params);

void JointState_t2JointStateRaw_t(JointState_t &orig,JointStateRaw_t &dest);

}

#endif
