#ifndef SERIALCOMMANDS_H
#define SERIALCOMMANDS_H

#include <modules/serialcom.h>

/*
 * Flavor-specific commands. The universal set (INFO, S_F, S_ON, S_MODE, DESC,
 * ...) comes from eriscommon; registerCommands() in serialcommand.cpp adds
 * these on top:
 *
 *   G0 [X<mm>] [F<mm/min>]   the G-code surface -- see gcode.h
 *   STEP                     dump the buffered axis telemetry as text
 *   TMC                      driver status; TMC I/U/S reconfigure it live
 *   SG [<n>|CLR]             StallGuard threshold: report, retune, clear latch
 *   EN <0|1>                 release / energize the motor
 *   ZERO                     call the current position 0 mm
 *   STOP                     decelerate to a halt, cancelling the move
 *
 * The file is serialcommandS.h, plural, on purpose. Windows filesystems are
 * case-insensitive, so a local "serialcommand.h" is found by eriscommon's
 * `#include <SerialCommand.h>` and shadows the library header -- see
 * lessons_learned.md section 5.
 */
namespace SerialCom {
    void TransmitStepper();
    void TMCCommand();
    void StallGuardCommand();
    void SetEnable();
    void ZeroPosition();
    void StopMotion();
}

#endif
