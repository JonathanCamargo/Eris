#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <modules/serialcom.h>

namespace SerialCom {
    void TransmitINA219();
    void ServoMove();
    void ServoSmoothMove();
}

#endif
