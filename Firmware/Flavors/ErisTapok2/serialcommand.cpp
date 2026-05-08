#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "sync.h"

#if SDCARD
#include "sdcard.h"
#endif

namespace SerialCom {

static void printEMGFields(const EMGSample_t& s)   { Serial.print(s.ch[0], 2); }
static void printFSRFields(const FSRSample_t& s)   { Serial.print(s.ch[0], 2); }
static void printSyncFields(const boolSample_t& s) { Serial.print(s.value); }

void TransmitEMG()  { transmitBuffer(EMG::buffer,  "EMG[ch0]",  printEMGFields); }
void TransmitFSR()  { transmitBuffer(FSR::buffer,  "FSR[ch0]",  printFSRFields); }
void TransmitSync() { transmitBuffer(Sync::buffer, "Sync",      printSyncFields); }

#if SDCARD
void TransmitEMG2() { transmitBuffer(SDCard::emgbuffer, "EMG[ch0]", printEMGFields); }
#else
void TransmitEMG2() { eriscommon::printText("SDCARD disabled in configuration.h"); }
#endif

void StartRecording() {
#if not SDCARD
    eriscommon::printText("SDCARD disabled in configuration.h");
#else
    char* arg = sCmd.next();
    if (arg != NULL) {
        SDCard::setTrialName(arg);
    } else {
        SDCard::setTrialName(SDCard::DEFAULT_TRIALNAME);
    }
    sprintf(strbuffer, "Start record on SDCARD (trial:%s)", SDCard::getTrialName());
    eriscommon::printText(strbuffer);
    SDCard::StartRecording();
#endif
}

void StopRecording() {
#if SDCARD
    SDCard::StopRecording();
#endif
    eriscommon::printText("Stop record on SDCard");
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("EMG",     TransmitEMG);
    sCmd.addCommand("EMG2",    TransmitEMG2);
    sCmd.addCommand("FSR",     TransmitFSR);
    sCmd.addCommand("SYNC",    TransmitSync);
    sCmd.addCommand("SD_REC",  StartRecording);
    sCmd.addCommand("SD_NREC", StopRecording);
}

} // namespace SerialCom
