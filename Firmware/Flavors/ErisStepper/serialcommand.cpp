#include "Eris.h"
#include "serialcommands.h"
#include "motion.h"
#include "tmc2209.h"
#include "gcode.h"

#include <stdlib.h>
#include <ctype.h>

namespace SerialCom {

static void printStepperFields(const StepperSample_t& s) {
    Serial.print(s.position, 3);
    Serial.print(":");
    Serial.print(s.velocity, 2);
    Serial.print(":");
    Serial.print(s.sg);
}

void TransmitStepper() {
    transmitBuffer(Stepper::buffer, "STEP", printStepperFields);
}

/*
 * TMC [I <mA> | U <microsteps> | S <0|1>]
 *
 * With no argument it dumps the driver status. The three sub-commands are the
 * reason the UART is wired at all: they change current, microstepping and
 * chopper mode while the motor is running, which a standalone TMC2209 wired to
 * MS1/MS2 and a trimpot cannot do.
 */
void TMCCommand() {
    char * what = sCmd.next();
    if (what == NULL){ TMC::printStatus(); return; }

    char * valueTok = sCmd.next();
    if (valueTok == NULL){
        eriscommon::println("TMC: I <mA> | U <microsteps> | S <0|1>");
        return;
    }
    long value = atol(valueTok);

    if (!TMC::present()){
        Error::RaiseError(Error::SENSOR, (char *)"TMC UART is down, cannot reconfigure");
        return;
    }

    switch (toupper((unsigned char)what[0])){
        case 'I':
            if (value <= 0 || value > 2000){
                Error::RaiseError(Error::COMMAND, (char *)"TMC I: give an rms current in mA (1..2000)");
                return;
            }
            TMC::setCurrent((uint16_t)value);
            sprintf(strbuffer, "TMC current set to %u mA rms", (unsigned)value);
            eriscommon::println(strbuffer);
            break;
        case 'U':
            // setMicrosteps() rejects a non power of two itself.
            if (TMC::setMicrosteps((uint16_t)value)){
                sprintf(strbuffer, "TMC microsteps set to %u", (unsigned)value);
                eriscommon::println(strbuffer);
                eriscommon::println("NOTE: STEPPER_STEPS_PER_MM now describes a different step size");
            }
            break;
        case 'S':
            TMC::setStealthChop(value != 0);
            eriscommon::println(value ? "TMC chopper: StealthChop" : "TMC chopper: SpreadCycle");
            break;
        default:
            eriscommon::println("TMC: I <mA> | U <microsteps> | S <0|1>");
            break;
    }
}

/*
 * SG [<0..255> | CLR]
 *
 * The hard-stop detector's threshold, and the only practical way to tune it.
 * With no argument it reports the live SG_RESULT next to the trip point, which
 * is what you watch while jogging the axis: run it unloaded, note the reading,
 * and set the threshold to somewhat under half of it (the driver trips at
 * SG_RESULT <= 2*SGTHRS). Push on the axis by hand and the reading should dive.
 */
void StallGuardCommand() {
#if !STEPPER_STALL_DETECT
    eriscommon::println("Stall detection is compiled out (STEPPER_STALL_DETECT 0)");
#else
    char * arg = sCmd.next();

    if (arg != NULL && (arg[0] == 'C' || arg[0] == 'c')){
        Motion::clearStall();
        eriscommon::println("Stall latch cleared");
        return;
    }

    if (arg != NULL){
        long value = atol(arg);
        if (value < 0 || value > 255){
            Error::RaiseError(Error::COMMAND, (char *)"SG: threshold is 0..255");
            return;
        }
        if (!TMC::present()){
            Error::RaiseError(Error::SENSOR, (char *)"TMC UART is down, cannot set SGTHRS");
            return;
        }
        TMC::setStallThreshold((uint8_t)value);
        sprintf(strbuffer, "SGTHRS set to %u (trips at sg<=%u)",
                (unsigned)value, (unsigned)(2 * value));
        eriscommon::println(strbuffer);
        return;
    }

    uint16_t sg = TMC::lastStallGuard();
    bool live = TMC::present() && TMC::readStallGuard(sg);
    sprintf(strbuffer, "sg:%s%u thrs:%u trips_at:%u armed:%u stalled:%u",
            live ? "" : "?", (unsigned)sg,
            (unsigned)TMC::stallThreshold(),
            (unsigned)(2 * TMC::stallThreshold()),
            (unsigned)(Motion::stallArmed() ? 1 : 0),
            (unsigned)(Motion::stalled() ? 1 : 0));
    eriscommon::println(strbuffer);
#endif
}

void SetEnable() {
    char * arg = sCmd.next();
    bool on = (arg == NULL) ? true : (atoi(arg) != 0);
    Motion::enable(on);
    eriscommon::println(on ? "Motor enabled" : "Motor released (position is no longer trustworthy)");
}

void ZeroPosition() {
    Motion::zero();
    eriscommon::println("Position zeroed");
}

void StopMotion() {
    Motion::stop();
    eriscommon::println("Stopping");
}

void registerCommands(SerialCommand& sCmd) {
    // SerialCommand matches the token exactly, so the lowercase spelling needs
    // its own registration -- senders and humans write both.
    sCmd.addCommand("G0",   GCode::G0);
    sCmd.addCommand("g0",   GCode::G0);
    sCmd.addCommand("STEP", TransmitStepper);
    sCmd.addCommand("TMC",  TMCCommand);
    sCmd.addCommand("SG",   StallGuardCommand);
    sCmd.addCommand("EN",   SetEnable);
    sCmd.addCommand("ZERO", ZeroPosition);
    sCmd.addCommand("STOP", StopMotion);
}

} // namespace SerialCom
