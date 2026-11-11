#include "Eris.h"
#include "loadcell.h"
#include "motion.h"
#include "HX711.h"

#include <math.h>

ERIS_PIN_ASSERT_OUTPUT(PIN_HX711_SCK);
ERIS_PIN_ASSERT_ADC1(PIN_FSR);

static_assert(CAL_POLL_HZ % CAL_FALLBACK_HZ == 0,
              "CAL_FALLBACK_HZ must divide CAL_POLL_HZ");
static_assert(CAL_POLL_HZ > 80,
              "CAL_POLL_HZ must out-run the HX711 (80 SPS max) or conversions are lost");

namespace Loadcell{

static HX711 hx;

// --- Calibration state -----------------------------------------------------
static volatile long  s_offset     = 0;
static volatile float s_countsPerN = LOADCELL_COUNTS_PER_N;
static volatile bool  s_calibrated = (LOADCELL_CALIBRATED != 0);
// False until the first TARE completes. Before that the offset is 0 and the
// cell's own bridge offset would read as a huge force, so the guard waits.
static volatile bool  s_tared      = false;

// --- Presence --------------------------------------------------------------
static volatile bool s_present  = false;
static bool          s_warned   = false;   // "no HX711" said once per absence
static uint32_t      s_lastConvMs = 0;
static uint32_t      s_bootMs     = 0;

// --- Latest values, for the LC command --------------------------------------
static volatile long  s_lastRaw     = 0;
static volatile float s_lastForce   = 0.0f;
static volatile float s_lastVoltage = 0.0f;

// --- Requests (written by the serial thread, served by the sampling thread) -
enum Request : uint8_t { REQ_NONE = 0, REQ_TARE, REQ_CAL };
static volatile uint8_t s_req     = REQ_NONE;
static float            s_knownN  = 0.0f;
static double           s_acc     = 0.0;   // double: 20 x 24-bit counts
static uint16_t         s_accN    = 0;

static const float FSR_COUNTS_FS = (float)((1UL << FSR_ADC_BITS) - 1);

static float readFSR(void){
  uint32_t sum = 0;
  for (uint8_t i = 0; i < FSR_OVERSAMPLE; i++) sum += (uint32_t)analogRead(PIN_FSR);
  return ((float)sum / FSR_OVERSAMPLE) * FSR_VREF / FSR_COUNTS_FS;
}

static inline float toForce(long raw){
  return (float)(raw - s_offset) / (s_calibrated ? s_countsPerN : 1.0f);
}

// Accumulate one fresh conversion toward the pending request, and finish it
// when enough have arrived. Runs on the sampling thread only.
static void serviceRequest(long raw){
  if (s_req == REQ_NONE) return;
  s_acc += (double)raw;
  if (++s_accN < LOADCELL_AVG_SAMPLES) return;

  double mean = s_acc / (double)s_accN;
  if (s_req == REQ_TARE){
    s_offset = (long)(mean + (mean >= 0 ? 0.5 : -0.5));
    s_tared  = true;
    eriscommon::print("TARE done, offset ");
    eriscommon::println((double)s_offset, 0);
  }
  else {  // REQ_CAL
    float cpn = (float)((mean - (double)s_offset) / (double)s_knownN);
    if (fabsf(cpn) < 1.0f){
      // Less than one count per newton is not a load cell under load: it is
      // a cell with nothing on it, a missed TARE, or a broken bridge wire.
      Error::RaiseError(Error::SENSOR,
        (char *)"LCAL: no load seen (tare first, then apply the known force)");
    }
    else {
      s_countsPerN = cpn;
      s_calibrated = true;
      eriscommon::print("LCAL done: ");
      eriscommon::print((double)cpn, 3);
      eriscommon::println(" counts/N -- put this in LOADCELL_COUNTS_PER_N");
    }
  }
  s_acc  = 0.0;
  s_accN = 0;
  s_req  = REQ_NONE;
}

// Overload guard with hysteresis. Only meaningful in newtons.
static void guard(float force){
  if (!s_calibrated || !s_tared) return;
  float f = fabsf(force);
  if (!Motion::pressBlocked() && f > CAL_FORCE_LIMIT_N){
    Motion::blockPress(true);
    Error::RaiseError(Error::SENSOR, (char *)"overload: axis halted, back off to clear");
    eriscommon::print("  force ");   eriscommon::print((double)force, 2);
    eriscommon::print(" N > limit "); eriscommon::println((double)CAL_FORCE_LIMIT_N, 1);
  }
  else if (Motion::pressBlocked() && f < CAL_FORCE_LIMIT_N * CAL_FORCE_RELEASE){
    Motion::blockPress(false);
    eriscommon::println("overload cleared");
  }
}

// Presence bookkeeping for a tick with no conversion. Returns true while the
// cell is still considered present (i.e. just between conversions).
static bool checkPresence(uint32_t now){
  if (s_present){
    if ((uint32_t)(now - s_lastConvMs) <= LOADCELL_TIMEOUT_MS) return true;
    s_present = false;
    s_warned  = true;
    // Fail safe: with no force reading the guard is blind, so stop pressing.
    // (Uncalibrated there is no guard to lose, and nothing would ever release
    // the block, so leave the axis alone.)
    if (s_calibrated){
      Motion::blockPress(true);
      Error::RaiseError(Error::SENSOR, (char *)"HX711 stopped converting -- pressing blocked");
    }
    else {
      Error::RaiseError(Error::SENSOR, (char *)"HX711 stopped converting");
    }
    return false;
  }
  if (!s_warned && (uint32_t)(now - s_bootMs) > LOADCELL_TIMEOUT_MS){
    s_warned = true;
    Error::RaiseError(Error::SENSOR, (char *)"no HX711 on DOUT/SCK -- streaming FSR only");
  }
  return false;
}

bool requestTare(void){
  if (s_req != REQ_NONE) return false;
  s_acc = 0.0; s_accN = 0;
  s_req = REQ_TARE;
  return true;
}

bool requestCalibration(float knownNewtons){
  if (s_req != REQ_NONE) return false;
  s_knownN = knownNewtons;
  s_acc = 0.0; s_accN = 0;
  s_req = REQ_CAL;
  return true;
}

bool  busy(void)       { return s_req != REQ_NONE; }
float scale(void)      { return s_countsPerN; }
long  offset(void)     { return s_offset; }
bool  calibrated(void) { return s_calibrated; }
bool  present(void)    { return s_present; }

void setScale(float countsPerN){
  s_countsPerN = countsPerN;
  s_calibrated = true;
}

void printStatus(void){
  eriscommon::print("HX711: ");      eriscommon::print(s_present ? "present" : "ABSENT");
  eriscommon::print("  raw ");       eriscommon::print((double)s_lastRaw, 0);
  eriscommon::print("  offset ");    eriscommon::print((double)s_offset, 0);
  eriscommon::print("  scale ");     eriscommon::print((double)s_countsPerN, 3);
  eriscommon::println(s_calibrated ? " counts/N" : " (UNCALIBRATED: force in counts)");
  eriscommon::print("force ");       eriscommon::print((double)s_lastForce, 3);
  eriscommon::print(s_calibrated ? " N" : " counts");
  eriscommon::print("  fsr ");       eriscommon::print((double)s_lastVoltage, 4);
  eriscommon::print(" V  guard ");
  if (!s_calibrated)               eriscommon::println("OFF");
  else if (Motion::pressBlocked()) eriscommon::println("HOLDING");
  else                             eriscommon::println("armed");
  if (busy()) eriscommon::println("(tare/calibration in progress)");
}

void begin(void){
#if !defined(ARDUINO_ARCH_AVR)
  analogReadResolution(FSR_ADC_BITS);
#endif
  pinMode(PIN_FSR, INPUT);

  hx.begin(PIN_HX711_DOUT, PIN_HX711_SCK, LOADCELL_GAIN);
  s_bootMs = millis();

  // Tare on boot: the first LOADCELL_AVG_SAMPLES conversions set the zero, so
  // the axis must not be pressing on the cell at power-up. If the cell is
  // absent the request simply waits until it shows up.
  requestTare();

  // Pressing is held off until a force reading exists, if the guard is in use.
  if (s_calibrated) Motion::blockPress(true);

  FSRCal::start();

  eriscommon::print("Load cell: HX711 @ ");
  eriscommon::print((double)LOADCELL_SPS, 0);
  eriscommon::print(" SPS, gain ");
  eriscommon::print((double)LOADCELL_GAIN, 0);
  if (s_calibrated){
    eriscommon::print(", ");  eriscommon::print((double)s_countsPerN, 3);
    eriscommon::print(" counts/N, limit "); eriscommon::print((double)CAL_FORCE_LIMIT_N, 1);
    eriscommon::println(" N");
  }
  else {
    eriscommon::println(", UNCALIBRATED (force in counts, overload guard off)");
  }
}

// Called from the sampling thread (defined below) -- kept in this namespace so
// it can reach the private state above.
bool fillSample(FSRCalSample_t & s){
  uint32_t now = millis();

  if (hx.is_ready()){
    // is_ready() was true, so read() does not block in wait_ready(). It does
    // disable interrupts for the ~25 clock pulses (well under 100 us).
    long raw = hx.read();
    s_lastConvMs = now;
    if (!s_present){
      s_present = true;
      if (s_warned){ s_warned = false; eriscommon::println("HX711 detected"); }
    }
    serviceRequest(raw);
    float f = toForce(raw);
    guard(f);

    s.raw   = (int32_t)raw;
    s.force = f;
    s.fresh = 1;
    s_lastRaw   = raw;
    s_lastForce = f;
  }
  else {
    if (checkPresence(now)) return false;   // between conversions: no sample
    // Absent: keep the FSR visible at a modest rate so the divider can be
    // checked on its own.
    static uint8_t div = 0;
    if (++div < (CAL_POLL_HZ / CAL_FALLBACK_HZ)) return false;
    div = 0;
    s.raw   = 0;
    s.force = 0.0f;
    s.fresh = 0;
  }

  s.voltage  = readFSR();
  s.position = Motion::positionMm();
  s.overload = Motion::pressBlocked() ? 1 : 0;
  s.pad0 = 0;
  s.pad1 = 0;
  s_lastVoltage = s.voltage;
  return true;
}

} // namespace Loadcell

// ---------------------------------------------------------------------------
// The calibration stream. Polls at CAL_POLL_HZ and emits a sample only when
// the HX711 has converted, so every row is a matched (voltage, force) pair.
//
// This bends ERIS_SENSOR slightly -- DOUT is a data-ready line -- but polling
// it at 200 Hz against an 80 SPS maximum can never miss a conversion (the
// HX711 holds the result until it is read), and the price is at most 5 ms of
// skew between force and voltage, which a quasi-static calibration does not
// notice. An interrupt on DOUT would not be better: it fires during the read's
// own clocking, which is why the library does not use one either.
//
// ERIS_SENSOR_EX for the LARGE stack: this thread raises errors and prints the
// tare/calibration results, both of which go through the packet layer.
// ---------------------------------------------------------------------------
ERIS_SENSOR_EX(FSRCal, FSRCalSample_t, CAL_POLL_HZ,
               ERIS_STACK_LARGE, ERIS_NORMAL_PRIORITY + 1) {
  return Loadcell::fillSample(s);
}
