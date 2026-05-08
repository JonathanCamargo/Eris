#include "Eris.h"
#include "serialcommand.h"

#include "biom.h"
#include "gait.h"
#include "features.h"

#include <FeatureExtractor.h>

namespace SerialCom {

bool classifQuery = false;

static void printBiomFields(const BiomSample_t& s) { Serial.print(s.ch[0], 2); }
void TransmitBiom() { transmitBuffer(Biom::buffer, "Biom[ch0]", printBiomFields); }

void TransmitGait() {
    Serial.print("Gait: ");
    Serial.println(Gait::currGait);
}

// Takes 1 arg: index of the feature to print
void TransmitFEAT() {
    char* arg = sCmd.next();
    if (arg != NULL) {
        uint8_t ind = (uint8_t)atoi(arg);
        if (ind < FEATS_NUM) {
            Serial.print("Features:");
            for (uint8_t i = 0; i < BIOM_NUMCHANNELS; i++) {
                Serial.print(Features::lastFeatures[i][ind], 2);
                if (i != BIOM_NUMCHANNELS - 1) Serial.print(",");
            }
            Serial.println();
        }
    }
}

void enableFeatures()  { Features::en_features = true; }
void disableFeatures() { Features::en_features = false; Features::clearExtractors(); }

// REG <window> <increment> <ambulation_mode>
void updateRegVals() {
    char* args[3];
    uint8_t index = 0;
    do {
        args[index] = sCmd.next();
        index++;
    } while (index < 3 && args[index] != NULL);
    if (index == 3 && args[2] != NULL) {
        uint16_t val1 = (uint16_t)atoi(args[0]);
        uint16_t val2 = (uint16_t)atoi(args[1]);
        uint8_t  val3 = (uint8_t)atoi(args[2]);

        eris_mutex_lock(&Features::extractMtx);
        Features::regWin = val1;
        Features::regInc = val2;
        Features::regIdx = val3;
        eris_mutex_unlock(&Features::extractMtx);
    }
}

// CLASS <window> <gait_location>
void getClassificationFeatures() {
    char* arg = sCmd.next();
    if (arg != NULL) {
        uint16_t win = (uint16_t)atoi(arg);
        arg = sCmd.next();
        if (arg != NULL) {
            uint8_t idx = (uint8_t)atoi(arg);
            eris_mutex_lock(&Features::extractMtx);
            Features::classIdx = idx;
            classifQuery = true;
            Features::extractHelper(win, 1);
            classifQuery = false;
            eris_mutex_unlock(&Features::extractMtx);
        }
    }
}

// F_MASK <j> <k> <l> <FEATS_NUM-char mask string of 0s and 1s>
void changeMask() {
    char* args[4];
    uint8_t index = 0;
    do {
        args[index] = sCmd.next();
        index++;
    } while (index < 4 && args[index] != NULL);
    if (args[3] != NULL && strlen(args[3]) == FEATS_NUM) {
        uint8_t j = atoi(args[0]);
        uint8_t k = atoi(args[1]);
        uint8_t l = atoi(args[2]);
        for (uint8_t i = 0; i < FEATS_NUM; i++) {
            Features::mask[j][k][l][i] = (bool)(args[3][i] - '0');
        }
    } else {
        eriscommon::printText("Error: wrong number of arguments or mask was the wrong size.");
    }
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("BIOM",   TransmitBiom);
    sCmd.addCommand("GAIT",   TransmitGait);
    sCmd.addCommand("FEAT",   TransmitFEAT);
    sCmd.addCommand("F_ON",   enableFeatures);
    sCmd.addCommand("F_OFF",  disableFeatures);
    sCmd.addCommand("F_MASK", changeMask);
    sCmd.addCommand("CLASS",  getClassificationFeatures);
    sCmd.addCommand("REG",    updateRegVals);
}

} // namespace SerialCom
