#ifndef SERVO_COMMANDS_H
#define SERVO_COMMANDS_H

#include <modules/serialcom.h>

namespace SerialCom {
    void ServoMove();
    void ServoSmoothMove();
}

#endif
