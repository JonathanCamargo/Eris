#include "Eris.h"
#include "serialcommand.h"
#include "servos.h"
#include "ina219.h"

namespace SerialCom {

static void printINA219Fields(const inaSample_t& s) {
    Serial.print(s.current_mA, 2);
    Serial.print("mA");
}

void TransmitINA219() {
    transmitBuffer(INA219::buffer, "INA219", printINA219Fields);
}

void ServoMove() {
    char *arg;
    float args[NUM_SERVOS];
    int nargs = 0;
    arg = sCmd.next();
    while (arg != NULL && nargs < NUM_SERVOS) { args[nargs++] = atof(arg); arg = sCmd.next(); }
    if (nargs == 2) {
        uint8_t channel = (uint8_t)args[0];
        float angle = args[1];
        Servos::move(channel, angle);
        Serial.print("Servo "); Serial.print(channel); Serial.print(" -> "); Serial.println(angle);
    } else if (nargs == NUM_SERVOS) {
        Servos::moveAll(args);
        Serial.println("All servos moved");
    } else {
        Serial.print("Error: expected 2 or "); Serial.print(NUM_SERVOS); Serial.println(" arguments");
    }
}

void ServoSmoothMove() {
    char *arg;
    float args[NUM_SERVOS];
    int nargs = 0;
    arg = sCmd.next();
    while (arg != NULL && nargs < NUM_SERVOS) { args[nargs++] = atof(arg); arg = sCmd.next(); }
    if (nargs == 2) {
        uint8_t channel = (uint8_t)args[0];
        float angle = args[1];
        Servos::smoothMove(channel, angle);
        Serial.print("Smooth servo "); Serial.print(channel); Serial.print(" -> "); Serial.println(angle);
    } else if (nargs == NUM_SERVOS) {
        Servos::smoothMoveAll(args);
        Serial.println("All servos smooth move");
    } else {
        Serial.print("Error: expected 2 or "); Serial.print(NUM_SERVOS); Serial.println(" arguments");
    }
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("INA", TransmitINA219);
    sCmd.addCommand("X",   ServoMove);
    sCmd.addCommand("Y",   ServoSmoothMove);
}

} // namespace SerialCom
