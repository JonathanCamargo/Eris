#include "Eris.h"
#include "serialcommand.h"
#include "analog.h"

namespace SerialCom {

static void printAnalogFields(const AnalogSample_t& s) {
    for (uint8_t c = 0; c < ANALOG_NUMCHANNELS; c++) {
        Serial.print(s.ch[c], 2);
        if (c < ANALOG_NUMCHANNELS - 1) Serial.print(",");        
    }
}

void TransmitAnalog() {
    transmitBuffer(analog.buffer, "Analog", printAnalogFields);
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("A", TransmitAnalog);
}

} // namespace SerialCom
