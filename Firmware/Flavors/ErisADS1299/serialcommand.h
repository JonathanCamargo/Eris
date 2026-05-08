#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <modules/serialcom.h>

namespace SerialCom {
    void TransmitEMG();
    void TransmitFSR();
    void TransmitSync();
    void StartRecording();
    void StopRecording();
}

#endif
