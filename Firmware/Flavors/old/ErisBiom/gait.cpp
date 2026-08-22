#include "Eris.h"
#include "gait.h"

#include "biom.h"
#include "configuration.h"
#include <CircularBuffer.h>

namespace Gait{

  long currTime = 0;
  long startTime = 0;
  CircularBuffer<long, 3> stepTime;
  float sumTime = 0;
  
  float currGait = 0;
  
  float riseFact;
  float currMin = -2;
  float currMax = -2;
  float rg;
  bool nextStep = true;
  
  float minHeight = 0.5;
  
  void updateGait(float data) {
  }

}
