#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "serialeti.h"

#if SDCARD
#include "sdcard.h"
#endif

namespace SerialCom {

static TISample_t tisamples[MEMBUFFERSIZE];

static void printEMGFields(const EMGSample_t& s) { Serial.print(s.ch[0], 2); }
static void printFSRFields(const FSRSample_t& s) { Serial.print(s.ch[0], 2); }

void TransmitEMG() { transmitBuffer(EMG::buffer, "EMG[ch0]", printEMGFields); }
void TransmitFSR() { transmitBuffer(FSR::buffer, "FSR[ch0]", printFSRFields); }

void TransmitETI() {
    ERIS_CRITICAL_ENTER();
    int num = SerialETI::buffer.FetchData(tisamples, (char*)"ETI", MEMBUFFERSIZE);
    long missed = SerialETI::buffer.missed();
    ERIS_CRITICAL_EXIT();

    Serial.print("Temperature[ch0]:");
    Serial.print("(missed:"); Serial.print(missed); Serial.print(") ");
    if (num > 0) {
        uint8_t i = 0;
        Serial.print("(@"); Serial.print(tisamples[0].timestamp, 2); Serial.print("ms)");
        for (i = 0; i < num - 1; i++) {
            Serial.print(tisamples[i].temperature[0], 2);
            Serial.print(",");
        }
        Serial.print(tisamples[i].temperature[0], 2);
        Serial.print("(@"); Serial.print(tisamples[i].timestamp, 2); Serial.println("ms)");
    } else {
        Serial.println();
    }

    Serial.print("Impedance[ch0]:");
    Serial.print("(missed:"); Serial.print(missed); Serial.print(") ");
    if (num > 0) {
        uint8_t i = 0;
        Serial.print("(@"); Serial.print(tisamples[0].timestamp, 2); Serial.print("ms)");
        for (i = 0; i < num - 1; i++) {
            Serial.print(tisamples[i].impedance[0], 2);
            Serial.print(",");
        }
        Serial.print(tisamples[i].impedance[0], 2);
        Serial.print("(@"); Serial.print(tisamples[i].timestamp, 2); Serial.println("ms)");
    } else {
        Serial.println();
    }
}

void StartRecording() {
#if not SDCARD
    eriscommon::print("SDCARD disabled in configuration.h");
#else
    char* arg = sCmd.next();
    if (arg != NULL) {
        SDCard::setTrialName(arg);
    } else {
        SDCard::setTrialName(SDCard::DEFAULT_TRIALNAME);
    }
    sprintf(strbuffer, "Start record on SDCARD (trial:%s)", SDCard::getTrialName());
    eriscommon::print(strbuffer);
    SDCard::StartRecording();
#endif
}

void StopRecording() {
#if SDCARD
    SDCard::StopRecording();
#endif
    eriscommon::print("Stop record on SDCard");
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("EMG",     TransmitEMG);
    sCmd.addCommand("FSR",     TransmitFSR);
    sCmd.addCommand("ETI",     TransmitETI);
    sCmd.addCommand("SD_REC",  StartRecording);
    sCmd.addCommand("SD_NREC", StopRecording);
}

} // namespace SerialCom
