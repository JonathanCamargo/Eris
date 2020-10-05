#ifndef JOINTS_H
#define JOINTS_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>
#include "customtypes.h"

namespace Joints{

extern ErisBuffer<JointStateSample_t> kneebuffer;
extern ErisBuffer<JointStateSample_t> anklebuffer;

#if DEBUG_TIME
extern  time_measurement_t time;
#endif

void start(void);
void kill(void);

double ImpedanceLawKnee(JointStateSample_t);
double ImpedanceLawAnkle(JointStateSample_t);

void ReadIncrementalData();

void UpdateState(JointStateSample_t, JointStateSample_t);
void UpdateCurrent_T(void *arg);
void SetIP(float k_Knee,float b_Knee, float theta_eq_Knee,float k_Ankle,float b_Ankle, float theta_eq_Ankle);
void SetIPA(float k, float b, float theta);
void SetIPK(float k, float b, float theta);
void SetIP(ImpedanceParameters_t paramsK, ImpedanceParameters_t paramsA);
void GetIP(float &k_Knee,float &b_Knee, float &theta_eq_Knee,float &k_Ankle,float &b_Ankle, float &theta_eq_Ankle);
void GetIP(ImpedanceParameters_t &paramsK, ImpedanceParameters_t &paramsA);

}

#endif
