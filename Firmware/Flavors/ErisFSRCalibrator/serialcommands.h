#ifndef SERIALCOMMANDS_H
#define SERIALCOMMANDS_H

#include <modules/serialcom.h>

/*
 * Flavor-specific commands. The universal set (INFO, S_F, S_ON, S_MODE, DESC,
 * ...) comes from eriscommon; registerCommands() in serialcommand.cpp adds:
 *
 *   G0 [X<mm>] [F<mm/min>]   move the axis -- see gcode.h
 *   EN <0|1>                 release / energize the motor
 *   ZERO                     call the current position 0 mm
 *   STOP                     decelerate to a halt, cancelling the move
 *   TARE                     zero the load cell (nothing touching it)
 *   LCAL <newtons>           calibrate with a known force on the cell
 *   LSCALE [counts/N]        report / set the load-cell scale directly
 *   LC                       load cell + FSR status
 *   CAL                      dump the buffered calibration samples as text
 *
 * The file is serialcommandS.h, plural, on purpose: on Windows a local
 * "serialcommand.h" shadows the SerialCommand library header (lessons_learned
 * section 5).
 */
namespace SerialCom {
    void SetEnable();
    void ZeroPosition();
    void StopMotion();
    void Tare();
    void LoadcellCalibrate();
    void LoadcellScale();
    void LoadcellStatus();
    void TransmitCal();
}

#endif
