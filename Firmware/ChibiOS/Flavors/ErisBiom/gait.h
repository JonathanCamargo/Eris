#ifndef GAIT_H
#define GAIT_H

#include <Arduino.h>

namespace Gait{

extern float currGait;
void updateGait(float data);

}

#endif
