#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "sync.h"

namespace SerialCom {

static void printEMGFields(const EMGSample_t& s)      { Serial.print(s.ch[0], 2); }
static void printFSRFields(const FSRSample_t& s)      { Serial.print(s.ch[0], 2); }
static void printSyncFields(const uint8_tSample_t& s) { Serial.print(s.value); }

void TransmitEMG()  { transmitBuffer(EMG::buffer,  "EMG[ch0]", printEMGFields); }
void TransmitFSR()  { transmitBuffer(FSR::buffer,  "FSR[ch0]", printFSRFields); }
void TransmitSync() { transmitBuffer(Sync::buffer, "Sync",     printSyncFields); }

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("EMG",  TransmitEMG);
    sCmd.addCommand("FSR",  TransmitFSR);
    sCmd.addCommand("SYNC", TransmitSync);
}

} // namespace SerialCom
