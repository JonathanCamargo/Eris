#ifndef IMU_H
#define IMU_H

#include "Eris.h"

#include <Arduino.h>

namespace IMU{

extern ErisBuffer<IMUSample_t> bufferTrunk;
extern ErisBuffer<IMUSample_t> bufferThigh;
extern ErisBuffer<IMUSample_t> bufferShank;
extern ErisBuffer<IMUSample_t> bufferFoot;
extern int failures;
void start(void);
void InitIMU(void);

}

#endif
