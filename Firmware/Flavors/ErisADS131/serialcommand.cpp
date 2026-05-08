#include "Eris.h"

namespace SerialCom {

static void printEMGFields(const EMGSample_t& s) {
    Serial.print(s.ch[0], 2);   // matches original "ch0 only" display
}

void TransmitEMG() { transmitBuffer(ADS131::buffer, "EMG", printEMGFields); }

void StartADC() {
    eriscommon::printText("Start collecting on ADC...\n");
    ADS131::startCollecting();
}

void StopADC() {
    ADS131::stopCollecting();
    eriscommon::printText("Stop collecting on ADC\n");
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("EMG",     TransmitEMG);
    sCmd.addCommand("SD_REC",  StartADC);   // legacy name — actually starts ADC, not SD
    sCmd.addCommand("SD_NREC", StopADC);
}

} // namespace SerialCom
