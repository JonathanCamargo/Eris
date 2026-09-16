#ifndef TMC2209_H
#define TMC2209_H

#include <Arduino.h>
#include "Eris.h"

/*
 * TMC2209 single-wire UART: the CONFIGURATION path.
 *
 * Motion never comes through here -- that is STEP/DIR (see motion.cpp). The
 * UART sets what a step MEANS: how much current the coil gets, how many
 * microsteps make a full step, whether the chopper runs in StealthChop or
 * SpreadCycle. It also reads back diagnostics, which is the other half of why
 * it is worth wiring: a standalone TMC2209 cannot tell you it is in thermal
 * shutdown, it just stops producing torque.
 *
 * This is a small hand-rolled driver rather than a dependency on TMCStepper.
 * Eris flavors are copied and edited by students, and the whole protocol is
 * three datagram shapes and a CRC -- worth having in the flavor where it can be
 * read, and it keeps the flavor compiling with nothing installed.
 *
 * THE PROTOCOL (datasheet section 4.3)
 *
 *   write:  0x05  NODEADDR  REG|0x80  data[4]  CRC      (8 bytes, no reply)
 *   read:   0x05  NODEADDR  REG       CRC               (4 bytes)
 *   reply:  0x05  0xFF      REG       data[4]  CRC      (8 bytes)
 *
 * Data is big-endian. CRC is the ATM CRC-8 (poly 0x07) over every preceding
 * byte. The reply is addressed to 0xFF (the master), which is how a reply is
 * told apart from the echo of our own request coming back down the shared wire.
 */

namespace TMC {

// Register addresses used here.
static const uint8_t REG_GCONF      = 0x00;
static const uint8_t REG_GSTAT      = 0x01;
static const uint8_t REG_IFCNT      = 0x02;
static const uint8_t REG_IOIN       = 0x06;
static const uint8_t REG_IHOLD_IRUN = 0x10;
static const uint8_t REG_TPOWERDOWN = 0x11;
static const uint8_t REG_TPWMTHRS   = 0x13;
static const uint8_t REG_TCOOLTHRS  = 0x14;
static const uint8_t REG_SGTHRS     = 0x40;
static const uint8_t REG_SG_RESULT  = 0x41;
static const uint8_t REG_CHOPCONF   = 0x6C;
static const uint8_t REG_DRV_STATUS = 0x6F;

// Packed DRV_STATUS, as carried in StepperSample_t.status. The raw register is
// 32 bits and mostly reserved; these are the bits worth streaming.
static const uint16_t ST_OTPW    = 1u << 0;   // overtemperature prewarning
static const uint16_t ST_OT      = 1u << 1;   // overtemperature shutdown
static const uint16_t ST_S2GA    = 1u << 2;   // short to ground, coil A
static const uint16_t ST_S2GB    = 1u << 3;   // short to ground, coil B
static const uint16_t ST_S2VSA   = 1u << 4;   // short to supply, coil A
static const uint16_t ST_S2VSB   = 1u << 5;   // short to supply, coil B
static const uint16_t ST_OLA     = 1u << 6;   // open load, coil A
static const uint16_t ST_OLB     = 1u << 7;   // open load, coil B
static const uint16_t ST_STEALTH = 1u << 8;   // StealthChop active
static const uint16_t ST_STST    = 1u << 9;   // standstill detected
// All 16 bits are spoken for (0..9 flags, 10..14 CS_ACTUAL, 15 link), which is
// why the stall latch rides in StepperSample_t::stalled rather than in here.
// bits 10..14: CS_ACTUAL, the current scaler the chip is actually applying
static const uint8_t  ST_CS_SHIFT = 10;
static const uint16_t ST_LINK    = 1u << 15;  // UART answered -- bits above are real

// Bring the UART up, verify the driver answers, and apply configuration.h.
// Returns false (and raises a SENSOR error) when nothing answers: STEP/DIR
// still works in that case, using whatever MS1/MS2 select in standalone mode,
// which is exactly the failure worth knowing about.
bool begin(void);

bool present(void);            // did the last connection test pass
uint8_t version(void);         // IOIN VERSION byte; 0x21 == TMC2209

// Raw register access. Both take the bus mutex, so they are safe from any
// thread -- but they block for up to TMC_REPLY_TIMEOUT_MS, so keep them off
// the motion thread.
bool writeRegister(uint8_t reg, uint32_t value);
bool readRegister(uint8_t reg, uint32_t & value);

// Configuration, applied immediately.
bool setCurrent(uint16_t mA);        // RMS mA, converted with TMC_RSENSE
bool setMicrosteps(uint16_t usteps); // 1..256, powers of two
bool setStealthChop(bool on);

// --- StallGuard4 ----------------------------------------------------------
// The load measurement the hard-stop detector thresholds. SG_RESULT is 0..1023
// and falls as mechanical load rises; the driver calls it a stall (and pulses
// DIAG) when SG_RESULT <= 2 * SGTHRS, so a HIGHER threshold trips EARLIER.
bool     setStallThreshold(uint8_t sgthrs);
uint8_t  stallThreshold(void);       // the shadow, no UART traffic
// One UART transaction. Returns false and leaves `sg` untouched if the read
// failed -- a dropped datagram must never read as a low (== stalled) value.
bool     readStallGuard(uint16_t & sg);
uint16_t lastStallGuard(void);       // cached, no UART traffic

uint16_t readStatus(void);     // one UART transaction -> packed ST_* bits
uint16_t lastStatus(void);     // cached, no UART traffic
void printStatus(void);        // human-readable dump for the TMC command

} // namespace TMC

#endif
