#include "Eris.h"
#include "serialcommands.h"
#include "motion.h"
#include "loadcell.h"
#include "gcode.h"

#include <stdlib.h>

namespace SerialCom {

// Parse a float argument, rejecting garbage: strtod alone reads "abc" as 0.
static bool parseFloat(const char * tok, float & out){
    if (tok == NULL) return false;
    char * end = NULL;
    double v = strtod(tok, &end);
    if (end == tok || *end != '\0') return false;
    out = (float)v;
    return true;
}

static void printCalFields(const FSRCalSample_t& s) {
    Serial.print(s.voltage, 4);
    Serial.print(":");
    Serial.print(s.force, 3);
    Serial.print(":");
    Serial.print(s.position, 3);
}

void TransmitCal() {
    transmitBuffer(FSRCal::buffer, "CAL", printCalFields);
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

void Tare() {
    if (!Loadcell::requestTare()){
        Error::RaiseError(Error::COMMAND, (char *)"TARE: a tare/calibration is already running");
        return;
    }
    if (!Loadcell::present())
        eriscommon::println("TARE queued -- runs when the HX711 starts converting");
    else
        eriscommon::println("TARE: averaging, keep the cell unloaded...");
}

/*
 * LCAL <newtons>
 *
 * Put a known force on the cell (a weight: newtons = kg * 9.80665), then run
 * this. Tare first with the cell unloaded -- LCAL measures relative to the
 * current offset.
 */
void LoadcellCalibrate() {
    float n;
    if (!parseFloat(sCmd.next(), n) || n == 0.0f){
        eriscommon::println("LCAL <newtons>: the known force now on the cell (kg x 9.80665)");
        return;
    }
    if (!Loadcell::present()){
        Error::RaiseError(Error::SENSOR, (char *)"LCAL: HX711 not present");
        return;
    }
    if (!Loadcell::requestCalibration(n)){
        Error::RaiseError(Error::COMMAND, (char *)"LCAL: a tare/calibration is already running");
        return;
    }
    eriscommon::println("LCAL: averaging, keep the load still...");
}

// LSCALE [counts/N] -- report, or restore a scale LCAL printed earlier.
void LoadcellScale() {
    char * arg = sCmd.next();
    if (arg == NULL){
        eriscommon::print("scale ");
        eriscommon::print((double)Loadcell::scale(), 3);
        eriscommon::println(Loadcell::calibrated() ? " counts/N" : " (uncalibrated)");
        return;
    }
    float v;
    if (!parseFloat(arg, v) || v == 0.0f){
        Error::RaiseError(Error::COMMAND, (char *)"LSCALE: give a non-zero counts/N");
        return;
    }
    Loadcell::setScale(v);
    eriscommon::print("scale set to ");
    eriscommon::print((double)v, 3);
    eriscommon::println(" counts/N (overload guard armed)");
}

void LoadcellStatus() {
    Loadcell::printStatus();
}

void registerCommands(SerialCommand& sCmd) {
    // SerialCommand matches tokens exactly, so g0 needs its own entry.
    sCmd.addCommand("G0",     GCode::G0);
    sCmd.addCommand("g0",     GCode::G0);
    sCmd.addCommand("EN",     SetEnable);
    sCmd.addCommand("ZERO",   ZeroPosition);
    sCmd.addCommand("STOP",   StopMotion);
    sCmd.addCommand("TARE",   Tare);
    sCmd.addCommand("LCAL",   LoadcellCalibrate);
    sCmd.addCommand("LSCALE", LoadcellScale);
    sCmd.addCommand("LC",     LoadcellStatus);
    sCmd.addCommand("CAL",    TransmitCal);
}

} // namespace SerialCom
