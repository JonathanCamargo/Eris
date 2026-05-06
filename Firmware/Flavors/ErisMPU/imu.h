#ifndef IMU_H
#define IMU_H

#include "Eris.h"

#include <Arduino.h>
#include "TimerTC3.h"

namespace IMU{

extern ErisBuffer<IMUSample_t> buffer0;
extern ErisBuffer<IMUSample_t> buffer1;
extern int failures;
void start(void);
void InitIMU(void);

}

#endif
