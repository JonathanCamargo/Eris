#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "serialselector.h"

namespace SerialCom {

static void printEMGFields(const EMGSample_t& s) { Serial.print(s.ch[0], 2); }
static void printFSRFields(const FSRSample_t& s) { Serial.print(s.ch[0], 2); }

void TransmitEMG() { transmitBuffer(EMG::buffer, "EMG[ch0]", printEMGFields); }
void TransmitFSR() { transmitBuffer(FSR::buffer, "FSR",      printFSRFields); }

void SelectNegativeElectrode() {
    ERIS_CRITICAL_ENTER();
    char* arg = sCmd.next();
    if (arg != NULL) {
        uint8_t index = atoi(arg);
        SerialSelector::SelectNegativeElectrode(index);
    }
    ERIS_CRITICAL_EXIT();
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("EMG", TransmitEMG);
    sCmd.addCommand("FSR", TransmitFSR);
    sCmd.addCommand("NEG", SelectNegativeElectrode);
}

} // namespace SerialCom
