#ifndef GCODE_H
#define GCODE_H

#include <Arduino.h>
#include "Eris.h"

/*
 * The G-code surface of this flavor, which is deliberately tiny:
 *
 *     G0 X<mm>            move to an absolute position
 *     G0 X<mm> F<mm/min>  ... and set the feedrate for this and later moves
 *     G0 F<mm/min>        set the feedrate only; nothing moves
 *     G0                  report the current position, target and feedrate
 *
 * Coordinates are absolute millimetres (G90 semantics, which is the default and
 * the only mode -- there is no G91). The feedrate is the G-code F word and is
 * therefore millimetres per MINUTE, not per second; it persists until changed,
 * exactly as it does on a real controller.
 *
 * G1 is not implemented. On a machine with one axis and no extruder G1 would be
 * a synonym for G0, and a silent synonym is worse than an unrecognized command:
 * a sender that emits G1 is a sender expecting coordinated multi-axis feed
 * moves, which this is not.
 *
 * Words are whitespace separated ("G0 X10 F600"), because Eris tokenizes the
 * command line on spaces before it gets here. "G0X10" will not parse.
 */
namespace GCode{

void G0(void);   // registered as the G0 / g0 serial command

}

#endif
