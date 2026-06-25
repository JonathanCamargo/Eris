#include "Eris.h"
#include "servo_commands.h"
#include "servos.h"

namespace SerialCom {

void ServoMove() {
    // X <ch> <angle>  OR  X <a0> <a1> ... <a15>
    char *arg;
    float args[NUM_SERVOS];
    int nargs = 0;

    arg = sCmd.next();
    while (arg != NULL && nargs < NUM_SERVOS) {
        args[nargs++] = atof(arg);
        arg = sCmd.next();
    }

    if (nargs == 2) {
        uint8_t channel = (uint8_t)args[0];
        float angle = args[1];
        Servos::move(channel, angle);
        Serial.print("Servo ");
        Serial.print(channel);
        Serial.print(" -> ");
        Serial.println(angle);
    } else if (nargs == NUM_SERVOS) {
        Servos::moveAll(args);
        Serial.println("All servos moved");
    } else {
        Serial.print("Error: expected 2 or ");
        Serial.print(NUM_SERVOS);
        Serial.println(" arguments");
    }
}

void ServoSmoothMove() {
    // Y <ch> <angle>  OR  Y <a0> <a1> ... <a15>
    char *arg;
    float args[NUM_SERVOS];
    int nargs = 0;

    arg = sCmd.next();
    while (arg != NULL && nargs < NUM_SERVOS) {
        args[nargs++] = atof(arg);
        arg = sCmd.next();
    }

    if (nargs == 2) {
        uint8_t channel = (uint8_t)args[0];
        float angle = args[1];
        Servos::smoothMove(channel, angle);
        Serial.print("Smooth servo ");
        Serial.print(channel);
        Serial.print(" -> ");
        Serial.println(angle);
    } else if (nargs == NUM_SERVOS) {
        Servos::smoothMoveAll(args);
        Serial.println("All servos smooth move");
    } else {
        Serial.print("Error: expected 2 or ");
        Serial.print(NUM_SERVOS);
        Serial.println(" arguments");
    }
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("X", ServoMove);
    sCmd.addCommand("Y", ServoSmoothMove);
}

} // namespace SerialCom
