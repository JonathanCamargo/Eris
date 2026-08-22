#include "Eris.h"
#include "serialcommand.h"

#include "biom.h"
#include "gait.h"
#include "features.h"
#include "sync.h"

namespace SerialCom {

static void printBiomFields(const BiomSample_t& s) { Serial.print(s.ch[0], 2); }
static void printSyncFields(const boolSample_t& s) { Serial.print(s.value); }

void TransmitBiom() { transmitBuffer(Biom::buffer, "Biom[ch0]", printBiomFields); }
void TransmitSync() { transmitBuffer(Sync::buffer, "Sync",      printSyncFields); }

void TransmitGait() {
    Serial.print("Gait: ");
    Serial.println(Gait::currGait);
}

void EnableFeatures()  { Features::en_features = true; }
void DisableFeatures() { Features::en_features = false; Features::clearExtractors(); }

void registerCommands(SerialCommand& sCmd) {
    // Data dump
    sCmd.addCommand("BIOM",   TransmitBiom);
    sCmd.addCommand("GAIT",   TransmitGait);
    sCmd.addCommand("SYNC",   TransmitSync);

    // Feature extraction (handlers in serialcommand_features.cpp, global ns)
    sCmd.addCommand("F_ON",    EnableFeatures);
    sCmd.addCommand("F_OFF",   DisableFeatures);
    sCmd.addCommand("F_CLASS", ::ClassifQuery);
    sCmd.addCommand("F_R",     ::Register);
    sCmd.addCommand("F_MASK",  ::ChangeMask);
    sCmd.addCommand("F_WIN",   ::ChangeWindow);
    sCmd.addCommand("F_IDX",   ::ChangeIdx);
    sCmd.addCommand("F_INFO",  ::FeaturesInfo);
}

} // namespace SerialCom
