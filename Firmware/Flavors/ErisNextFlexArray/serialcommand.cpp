#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "serialselector.h"

#if SDCARD
#include "sdcard.h"
#endif

namespace SerialCom {

void TransmitEMG() {
    ERIS_CRITICAL_ENTER();
    float values[MEMBUFFERSIZE];
    int num = EMG::buffer[0]->FetchData(values, (char*)"EMG", MEMBUFFERSIZE);
    long missed = EMG::buffer[0]->missed();
    ERIS_CRITICAL_EXIT();

    eriscommon::print("EMG:");
    eriscommon::print("("); eriscommon::print(missed); eriscommon::print(") ");
    if (num > 0) {
        for (uint8_t i = 0; i < num - 1; i++) {
            eriscommon::print(values[i], 2);
            eriscommon::print(",");
        }
        eriscommon::println(values[num - 1], 2);
    } else {
        eriscommon::println();
    }
}

void TransmitFSR() {
    ERIS_CRITICAL_ENTER();
    float values[MEMBUFFERSIZE];
    int num = FSR::buffer[0]->FetchData(values, (char*)"FSR", MEMBUFFERSIZE);
    long missed = FSR::buffer[0]->missed();
    ERIS_CRITICAL_EXIT();

    eriscommon::print("FSR:");
    eriscommon::print("("); eriscommon::print(missed); eriscommon::print(") ");
    if (num > 0) {
        for (uint8_t i = 0; i < num - 1; i++) {
            eriscommon::print(values[i], 2);
            eriscommon::print(",");
        }
        eriscommon::println(values[num - 1], 2);
    } else {
        eriscommon::println();
    }
}

void SelectNegativeElectrode() {
    ERIS_CRITICAL_ENTER();
    char* arg = sCmd.next();
    if (arg != NULL) {
        uint8_t index = atoi(arg);
        SerialSelector::SelectNegativeElectrode(index);
    }
    ERIS_CRITICAL_EXIT();
}

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
    sCmd.addCommand("FSR",     TransmitFSR);
    sCmd.addCommand("NEG",     SelectNegativeElectrode);
    sCmd.addCommand("SD_REC",  StartRecording);
    sCmd.addCommand("SD_NREC", StopRecording);
}

} // namespace SerialCom
