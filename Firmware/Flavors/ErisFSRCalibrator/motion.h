#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

#include "Eris.h"
#include <eriscommon.h>

/*
 * Motion: one stepper axis behind an A4988 (STEP/DIR/EN).
 *
 * Derived from ErisStepper's motion thread with the TMC2209 UART and the
 * StallGuard machinery removed -- an A4988 has neither. What it adds instead is
 * a press guard, driven by the load cell: see blockPress().
 *
 * Millimetres at the edges, steps on the inside. STEPPER_STEPS_PER_MM in
 * configuration.h is the only conversion. No homing: position 0 is wherever
 * the axis was at power-up, or wherever ZERO was last issued.
 */
namespace Motion{

void begin(void);                 // configure pins, start the motion thread

// G-code entry points ------------------------------------------------------
// moveTo() is what G0 X<mm> calls. Returns false, and moves nothing, when the
// target is outside the soft limits, the motor is released, or the press
// guard is holding the axis and the target is in the pressing direction.
bool  moveTo(float mm);
// Feedrate in mm/min (the G-code F word), clamped; returns the value adopted.
float setFeed(float mmPerMin);
float feed(void);

// Manual control -----------------------------------------------------------
void  stop(void);                 // decelerate to a halt at the current rate
void  halt(void);                 // stop dead, no ramp (overload)
void  zero(void);                 // call the current position 0 mm
void  enable(bool on);            // energize / release the motor

// Press guard --------------------------------------------------------------
// While blocked, the axis refuses any move in CAL_PRESS_DIR. Moves the other
// way (backing off the load) are always allowed -- refusing those would leave
// the probe stuck against the load cell. Driven by the Loadcell sensor.
void  blockPress(bool on);
bool  pressBlocked(void);

// State --------------------------------------------------------------------
float positionMm(void);
float targetMm(void);
float velocityMmS(void);          // signed: negative means travelling toward -X
bool  isMoving(void);
bool  isEnabled(void);

}

#endif
