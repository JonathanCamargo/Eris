#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <modules/serialcom.h>

namespace SerialCom {
    void TransmitFSR();
    void TransmitSync();
    void TransmitJointState();
    void TransmitLoadcellState();
    void SetIP();
    void SetIPA();
    void SetIPK();
    void GetIP();
}

#endif
