#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <modules/serialcom.h>

namespace SerialCom {
    void TransmitBiom();
    void TransmitGait();
    void TransmitFEAT();
    void enableFeatures();
    void disableFeatures();
    void updateRegVals();
    void getClassificationFeatures();
    void changeMask();
}

#endif
