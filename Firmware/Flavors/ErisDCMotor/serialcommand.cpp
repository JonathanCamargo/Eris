#include "Eris.h"
#include "serialcommand.h"
#include "motor.h"
#include "potentiometer.h"

namespace SerialCom {

static void printPotentiometerFields(const PotentiometerSample_t& s) {
    for (uint8_t c = 0; c < POT_NUMCHANNELS; c++) {
        Serial.print(s.ch[c], 2);
        if (c < POT_NUMCHANNELS - 1) Serial.print(",");
    }
}

void TransmitPotentiometer() {
    transmitBuffer(Potentiometer::buffer, "Potentiometer", printPotentiometerFields);
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("POT",      TransmitPotentiometer);
    sCmd.addCommand("MOT_F",    Motor::Forward);
    sCmd.addCommand("MOT_B",    Motor::Backward);
    sCmd.addCommand("MOT_STOP", Motor::Break);
    sCmd.addCommand("MOT_OFF",  Motor::Idle);
}

} // namespace SerialCom
