#include "Eris.h"
#include "serialcommands.h"
#include "emgpill.h"

namespace SerialCom {

static void printEMGFields(const EMGPillSample_t& s) {
    for (uint8_t i = 0; i < EMG_NUMCHANNELS; i++) {
        if (i) Serial.print(":");
        Serial.print(s.ch[i], 4);
    }
}

void TransmitEMG() {
    transmitBuffer(EMGPill::buffer, "EMG", printEMGFields);
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("EMG", TransmitEMG);
}

} // namespace SerialCom
