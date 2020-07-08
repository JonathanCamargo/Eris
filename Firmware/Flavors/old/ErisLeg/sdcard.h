#ifndef SDCARD_H
#define SDCARD_H


#include "Eris.h"

namespace SDCard{
  
void addSine(float);
void addJoint(JointState_t);
void addLoadcell(LoadcellData_t);

void start(void);
void StopRecording(void);
void StartRecording(void);

}

#endif
