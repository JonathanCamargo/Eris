#ifndef SERVOS_H
#define SERVOS_H

#include "Eris.h"

namespace Servos{
  void start();
  void move(uint8_t channel, int angle);
  void moveAll(int* angles);
  void smoothMove(uint8_t channel, int angle);
  void smoothMoveAll(int* angles);
}

#endif
