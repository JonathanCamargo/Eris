#include "Eris.h"
#include "tmc2209.h"

namespace TMC {

static eris_mutex_t busMutex;
static bool     linkOk      = false;
static uint8_t  chipVersion = 0;
static uint16_t statusCache = 0;
static uint8_t  sgthrsShadow = TMC_SGTHRS;
static uint16_t sgCache      = 0;

// Last known-good copies, so a read-modify-write still does the right thing on
// a link that has gone quiet. The values are the chip reset defaults.
static uint32_t gconfShadow    = 0x00000101;
static uint32_t chopconfShadow = 0x10000053;

// ---------------------------------------------------------------------------
// CRC-8/ATM, LSB first (datasheet 4.3, swuart_calcCRC)
// ---------------------------------------------------------------------------
static uint8_t crc8(const uint8_t * data, uint8_t len){
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++){
    uint8_t b = data[i];
    for (uint8_t j = 0; j < 8; j++){
      if ((crc >> 7) ^ (b & 0x01)) crc = (uint8_t)((crc << 1) ^ 0x07);
      else                         crc = (uint8_t)(crc << 1);
      b >>= 1;
    }
  }
  return crc;
}

static void flushInput(void){
  while (TMC_UART.available()) TMC_UART.read();
}

/*
 * Wait a moment for the next UART byte.
 *
 * Busy-wait, deliberately: one byte at 115200 baud is ~87 us and a whole
 * datagram is ~0.7 ms, so sleeping an RTOS tick per byte would stretch every
 * register access to 8 ms or more. Polling at TMC_POLL_US keeps a transaction
 * inside a single millisecond. The motion thread runs at a higher priority and
 * preempts this regardless, so the spin costs nothing that matters.
 *
 * Historical note worth keeping: this used eris_sleep_ms(1), which on the
 * FreeRTOS boards was called from start() before the scheduler existed and
 * hard-faulted on a NULL pxCurrentTCB. start() now runs in a task of its own
 * (see eris_scheduler_start in eris_rtos.h, and lessons_learned.md section 18),
 * so sleeping here would be legal again -- it is just the wrong granularity.
 */
static inline void tmcWaitByte(void){
  delayMicroseconds(TMC_POLL_US);
}

// ---------------------------------------------------------------------------
// Datagrams
// ---------------------------------------------------------------------------
static bool writeRegisterRaw(uint8_t reg, uint32_t value){
  uint8_t d[8];
  d[0] = 0x05;
  d[1] = TMC_ADDRESS;
  d[2] = (uint8_t)(reg | 0x80);        // MSB set = write
  d[3] = (uint8_t)(value >> 24);
  d[4] = (uint8_t)(value >> 16);
  d[5] = (uint8_t)(value >> 8);
  d[6] = (uint8_t)(value);
  d[7] = crc8(d, 7);

  flushInput();
  TMC_UART.write(d, sizeof(d));
  TMC_UART.flush();                    // wait for the last stop bit
  // The driver does not answer a write, but our own bytes come back down the
  // shared wire. Drop them so they cannot be mistaken for the next reply.
  uint32_t deadline = millis() + TMC_REPLY_TIMEOUT_MS;
  uint8_t seen = 0;
  while (seen < sizeof(d) && (int32_t)(millis() - deadline) < 0){
    if (TMC_UART.available()) { TMC_UART.read(); seen++; }
    else tmcWaitByte();
  }
  return true;
}

static bool readRegisterRaw(uint8_t reg, uint32_t & value){
  uint8_t req[4];
  req[0] = 0x05;
  req[1] = TMC_ADDRESS;
  req[2] = reg;                        // MSB clear = read
  req[3] = crc8(req, 3);

  flushInput();
  TMC_UART.write(req, sizeof(req));
  TMC_UART.flush();

  /*
   * Do not "discard the first 4 bytes": on a one-wire bus the echo is there,
   * on a bus with a real half-duplex transceiver it is not, and a fixed
   * discard turns the second case into a silent framing bug. Instead resync on
   * the pair that only a reply can start with -- 0x05 followed by 0xFF, the
   * master address. Our own request echoes back as 0x05 followed by the NODE
   * address, so it never matches.
   */
  uint8_t  buf[8];
  uint8_t  n = 0;
  uint32_t deadline = millis() + TMC_REPLY_TIMEOUT_MS;

  while (n < sizeof(buf) && (int32_t)(millis() - deadline) < 0){
    if (!TMC_UART.available()){ tmcWaitByte(); continue; }
    uint8_t b = (uint8_t)TMC_UART.read();
    if (n == 0){
      if (b == 0x05) buf[n++] = b;
    }
    else if (n == 1){
      if      (b == 0xFF) buf[n++] = b;   // reply frame
      else if (b == 0x05) n = 1;          // that was a new sync byte
      else                n = 0;
    }
    else {
      buf[n++] = b;
    }
  }

  if (n < sizeof(buf))          return false;   // timed out
  if (crc8(buf, 7) != buf[7])   return false;   // corrupt -- usually wiring
  if (buf[2] != reg)            return false;   // answer to a different read

  value = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
          ((uint32_t)buf[5] << 8)  |  (uint32_t)buf[6];
  return true;
}

// ---------------------------------------------------------------------------
// Public register access (mutex-guarded: the telemetry thread and the command
// thread both read, and two interleaved datagrams are two corrupt datagrams)
// ---------------------------------------------------------------------------
bool writeRegister(uint8_t reg, uint32_t value){
  eris_mutex_lock(&busMutex);
  bool ok = writeRegisterRaw(reg, value);
  eris_mutex_unlock(&busMutex);
  return ok;
}

bool readRegister(uint8_t reg, uint32_t & value){
  eris_mutex_lock(&busMutex);
  bool ok = readRegisterRaw(reg, value);
  eris_mutex_unlock(&busMutex);
  return ok;
}

bool present(void)        { return linkOk; }
uint8_t version(void)     { return chipVersion; }
uint16_t lastStatus(void) { return statusCache; }

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
bool setCurrent(uint16_t mA){
  /*
   * Datasheet current formula, solved for the 5-bit scaler:
   *
   *   I_rms = (CS+1)/32 * V_fs/(R_sense+0.02) * 1/sqrt(2)
   *
   * V_fs is 0.325 V normally and 0.180 V with VSENSE set. A CS below 16 means
   * the top half of the scale is unused, so switching to the low-sense range
   * buys back a bit of resolution -- which is why this computes it twice.
   */
  float cs = 32.0f * 1.41421f * (mA / 1000.0f) * (TMC_RSENSE + 0.02f) / 0.325f - 1.0f;
  bool  vsense = false;
  if (cs < 16.0f){
    vsense = true;
    cs = 32.0f * 1.41421f * (mA / 1000.0f) * (TMC_RSENSE + 0.02f) / 0.180f - 1.0f;
  }
  if (cs < 0.0f)  cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;

  uint8_t irun  = (uint8_t)(cs + 0.5f);
  uint8_t ihold = (uint8_t)(irun * TMC_HOLD_MULTIPLIER);

  if (vsense) chopconfShadow |=  (1UL << 17);
  else        chopconfShadow &= ~(1UL << 17);

  bool ok = writeRegister(REG_CHOPCONF, chopconfShadow);
  ok = writeRegister(REG_IHOLD_IRUN,
                     (uint32_t)ihold | ((uint32_t)irun << 8) | (10UL << 16)) && ok;
  ok = writeRegister(REG_TPOWERDOWN, TMC_POWERDOWN_DELAY) && ok;
  return ok;
}

bool setMicrosteps(uint16_t usteps){
  // MRES counts down from 256: 0->256, 1->128, ... 8->full step.
  uint8_t mres;
  switch (usteps){
    case 256: mres = 0; break;
    case 128: mres = 1; break;
    case  64: mres = 2; break;
    case  32: mres = 3; break;
    case  16: mres = 4; break;
    case   8: mres = 5; break;
    case   4: mres = 6; break;
    case   2: mres = 7; break;
    case   1: mres = 8; break;
    default:
      Error::RaiseError(Error::COMMAND,
        (char *)"TMC microsteps must be a power of two, 1..256");
      return false;
  }
  chopconfShadow &= ~(0x0FUL << 24);
  chopconfShadow |=  ((uint32_t)mres << 24);
  chopconfShadow |=  (1UL << 28);        // INTPOL: interpolate to 256 internally
  return writeRegister(REG_CHOPCONF, chopconfShadow);
}

bool setStealthChop(bool on){
  if (on) gconfShadow &= ~(1UL << 2);    // en_SpreadCycle = 0
  else    gconfShadow |=  (1UL << 2);
  return writeRegister(REG_GCONF, gconfShadow);
}

// ---------------------------------------------------------------------------
// StallGuard4
// ---------------------------------------------------------------------------
bool setStallThreshold(uint8_t sgthrs){
  sgthrsShadow = sgthrs;
  return writeRegister(REG_SGTHRS, (uint32_t)sgthrs);
}

uint8_t stallThreshold(void) { return sgthrsShadow; }

bool readStallGuard(uint16_t & sg){
  uint32_t v;
  // A failed read must NOT come back as a low number: low means "stalled", so
  // a dropped datagram would slam the axis to a halt. Report the failure and
  // leave the caller's value alone instead.
  if (!readRegister(REG_SG_RESULT, v)) return false;
  sgCache = (uint16_t)(v & 0x03FF);    // 10 bits
  sg = sgCache;
  return true;
}

uint16_t lastStallGuard(void) { return sgCache; }

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
uint16_t readStatus(void){
  uint32_t drv;
  if (!readRegister(REG_DRV_STATUS, drv)){
    statusCache = 0;                     // ST_LINK clear == nothing below is real
    return statusCache;
  }
  uint16_t s = ST_LINK;
  if (drv & (1UL <<  0)) s |= ST_OTPW;
  if (drv & (1UL <<  1)) s |= ST_OT;
  if (drv & (1UL <<  2)) s |= ST_S2GA;
  if (drv & (1UL <<  3)) s |= ST_S2GB;
  if (drv & (1UL <<  4)) s |= ST_S2VSA;
  if (drv & (1UL <<  5)) s |= ST_S2VSB;
  if (drv & (1UL <<  6)) s |= ST_OLA;
  if (drv & (1UL <<  7)) s |= ST_OLB;
  if (drv & (1UL << 30)) s |= ST_STEALTH;
  if (drv & (1UL << 31)) s |= ST_STST;
  s |= (uint16_t)(((drv >> 16) & 0x1F) << ST_CS_SHIFT);   // CS_ACTUAL
  statusCache = s;
  return s;
}

void printStatus(void){
  uint32_t drv  = 0;
  uint32_t ioin = 0;
  bool haveIoin = readRegister(REG_IOIN, ioin);
  bool haveDrv  = readRegister(REG_DRV_STATUS, drv);

  sprintf(strbuffer, "TMC link:%s version:0x%02X addr:%u",
          haveIoin ? "OK" : "DOWN", (unsigned)((ioin >> 24) & 0xFF),
          (unsigned)TMC_ADDRESS);
  eriscommon::println(strbuffer);

  if (!haveDrv){
    eriscommon::println("TMC DRV_STATUS unavailable (check PDN_UART wiring, 1k series resistor, MS1/MS2 address)");
    return;
  }

  uint16_t s = readStatus();
  sprintf(strbuffer, "TMC DRV_STATUS:0x%08lX cs:%u %s%s%s%s%s%s%s%s",
          (unsigned long)drv,
          (unsigned)((s >> ST_CS_SHIFT) & 0x1F),
          (s & ST_STST)    ? "standstill " : "",
          (s & ST_STEALTH) ? "stealth "    : "",
          (s & ST_OTPW)    ? "OTPW "       : "",
          (s & ST_OT)      ? "OT "         : "",
          (s & (ST_S2GA  | ST_S2GB))  ? "SHORT_GND " : "",
          (s & (ST_S2VSA | ST_S2VSB)) ? "SHORT_VS "  : "",
          (s & (ST_OLA   | ST_OLB))   ? "OPEN_LOAD " : "",
          (s & (ST_OT | ST_OTPW | ST_S2GA | ST_S2GB)) ? "" : "ok");
  eriscommon::println(strbuffer);

#if STEPPER_STALL_DETECT
  uint16_t sg = 0;
  bool haveSg = readStallGuard(sg);
  sprintf(strbuffer, "TMC stallguard sg:%s%u thrs:%u (trips at sg<=%u)",
          haveSg ? "" : "?", (unsigned)(haveSg ? sg : lastStallGuard()),
          (unsigned)sgthrsShadow, (unsigned)(2 * sgthrsShadow));
  eriscommon::println(strbuffer);
#endif
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------
bool begin(void){
  eris_mutex_init(&busMutex);
  ERIS_TMC_UART_BEGIN();
  delay(10);

  // Connection test. IOIN carries a hard-wired VERSION byte, so a correct
  // value proves the whole path: baud, address, CRC, and the series resistor.
  uint32_t ioin = 0;
  linkOk = readRegister(REG_IOIN, ioin);
  chipVersion = (uint8_t)((ioin >> 24) & 0xFF);

  if (!linkOk){
    Error::RaiseError(Error::SENSOR,
      (char *)"TMC2209 UART not answering -- driver stays in standalone mode");
    eriscommon::println("TMC2209 unconfigured: current/microsteps are whatever VREF and MS1/MS2 say");
    return false;
  }
  if (chipVersion != 0x21){
    sprintf(strbuffer, "TMC2209 answered with VERSION 0x%02X (expected 0x21)",
            (unsigned)chipVersion);
    Error::RaiseError(Error::SENSOR, strbuffer);
  }

  // Read-modify-write from the chip's own values, so anything not mentioned
  // here keeps its reset default instead of being clobbered by a literal.
  uint32_t v;
  if (readRegister(REG_GCONF, v))    gconfShadow    = v;
  if (readRegister(REG_CHOPCONF, v)) chopconfShadow = v;

  gconfShadow &= ~(1UL << 0);   // I_scale_analog=0: current comes from IRUN,
                                // not the VREF trimpot, so TMC_RMS_CURRENT_MA
                                // means what it says
  gconfShadow |=  (1UL << 6);   // pdn_disable=1: PDN_UART is a UART pin now
  gconfShadow |=  (1UL << 7);   // mstep_reg_select=1: microsteps from CHOPCONF,
                                // overriding MS1/MS2
  gconfShadow |=  (1UL << 8);   // multistep_filt=1: smooth a jittery STEP source
  gconfShadow &= ~(1UL << 3);   // shaft=0. STEPPER_INVERT_DIR is applied on the
                                // DIR pin instead (motion.cpp), so it still
                                // works when the UART is not wired up -- doing
                                // it in both places would cancel out
  writeRegister(REG_GCONF, gconfShadow);

  chopconfShadow &= ~0x0FUL;
  chopconfShadow |=  0x05UL;    // TOFF=5: chopper on. TOFF=0 disables the driver
  writeRegister(REG_CHOPCONF, chopconfShadow);

  setMicrosteps(TMC_MICROSTEPS);
  setCurrent(TMC_RMS_CURRENT_MA);
  setStealthChop(TMC_STEALTHCHOP);

#if STEPPER_STALL_DETECT
  // TCOOLTHRS is what switches StallGuard (and the DIAG output) on; leave it
  // at its reset value of 0 and SG_RESULT stays put at 0 forever, which looks
  // exactly like a permanently stalled motor.
  writeRegister(REG_TCOOLTHRS, TMC_TCOOLTHRS);
  setStallThreshold(TMC_SGTHRS);
#if !TMC_STEALTHCHOP
  eriscommon::println("WARNING: stall detection is on but StealthChop is off -- StallGuard4 is specified for StealthChop and will read noise");
#endif
#endif

  writeRegister(REG_GSTAT, 0x07);        // clear latched reset/error flags

  // IFCNT increments once per accepted write datagram. A link that reads but
  // never counts a write is the classic "RX works, TX resistor missing" fault,
  // and it would otherwise look like a perfectly configured driver.
  uint32_t ifcnt = 0;
  if (readRegister(REG_IFCNT, ifcnt) && ifcnt == 0){
    Error::RaiseError(Error::SENSOR,
      (char *)"TMC2209 accepted no writes (IFCNT=0) -- check the 1k TX resistor");
  }

  readStatus();
  sprintf(strbuffer, "TMC2209 ready: %u mA rms, %u microsteps, %s",
          (unsigned)TMC_RMS_CURRENT_MA, (unsigned)TMC_MICROSTEPS,
          TMC_STEALTHCHOP ? "StealthChop" : "SpreadCycle");
  eriscommon::println(strbuffer);
  return true;
}

} // namespace TMC
