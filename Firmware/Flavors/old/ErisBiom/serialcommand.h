#ifndef serialCOMMAND_H
#define serialCOMMAND_H

#include <modules/serialcom.h>
#include "serialcommand_features.h"   // F_R, F_MASK, F_WIN, F_IDX, F_INFO, F_CLASS handlers

namespace SerialCom {
    void TransmitBiom();
    void TransmitGait();
    void TransmitSync();
    void EnableFeatures();
    void DisableFeatures();
}

#endif
