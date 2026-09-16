#include "Eris.h"
#include "motion.h"
#include "tmc2209.h"

// Catch the ESP32-only pin mistakes at compile time (no-ops elsewhere).
ERIS_PIN_ASSERT_OUTPUT(PIN_TMC_STEP);
ERIS_PIN_ASSERT_OUTPUT(PIN_TMC_DIR);
ERIS_PIN_ASSERT_OUTPUT(PIN_TMC_EN);

namespace Motion{

// --- Units -----------------------------------------------------------------
// Everything below the API boundary is in steps and steps/s. Millimetres exist
// only at the edges, because integer steps are what the hardware actually does
// and float millimetres accumulate error over a long run of moves.
static const float TICK_S       = STEPPER_TICK_MS / 1000.0f;
static const float ACCEL_STEPS  = STEPPER_ACCEL * STEPPER_STEPS_PER_MM;      // steps/s^2
static const float MIN_VELOCITY = STEPPER_MIN_FEED * STEPPER_STEPS_PER_MM / 60.0f;
// Top rate the burst scheme can sustain, see the comment on the thread below.
static const float MAX_VELOCITY = (float)STEPPER_MAX_STEPS_PER_TICK / TICK_S;

static inline float toSteps(float mm){ return mm * STEPPER_STEPS_PER_MM; }
static inline float toMm(float steps){ return steps / STEPPER_STEPS_PER_MM; }

// --- State -----------------------------------------------------------------
// s_position is written only by the motion thread; s_target only by commands.
// Readers of either take a critical section, because a long is not written
// atomically on every board Eris runs on.
static volatile long   s_position = 0;      // steps
static volatile long   s_target   = 0;      // steps
static volatile float  s_velocity = 0.0f;   // steps/s, magnitude only
static volatile int8_t s_dir      = 1;      // direction latched on the DIR pin
static volatile bool   s_enabled  = false;

static float s_feedSteps = STEPPER_DEFAULT_FEED * STEPPER_STEPS_PER_MM / 60.0f;
static float s_frac      = 0.0f;            // sub-step carried between ticks

// Hard-stop detection state. s_stall is the latch; s_stallReported lets the
// telemetry thread do the talking, because printing from the 1 ms motion
// thread would blow both its tick budget and its stack.
static const float STALL_MIN_VELOCITY =
    STEPPER_STALL_MIN_FEED * STEPPER_STEPS_PER_MM / 60.0f;
static volatile bool     s_stall         = false;
static volatile bool     s_stallReported = true;
static volatile uint32_t s_moveStartMs   = 0;

static eris_thread_ref_t motionThread = NULL;

static inline void applyDir(int8_t dir){
  bool level = (dir > 0);
#if STEPPER_INVERT_DIR
  level = !level;
#endif
  digitalWrite(PIN_TMC_DIR, level ? HIGH : LOW);
  // DIR must be stable before the next STEP edge. The TMC2209 asks for 20 ns;
  // one microsecond is free here and survives any level shifting in between.
  delayMicroseconds(1);
}

/*
 * Sleep to the next tick boundary, resyncing if we overran this one.
 *
 * eris_sleep_until() returns immediately when the deadline has already passed
 * (that is its contract, on both RTOSes). For a thread running at
 * ERIS_NORMAL_PRIORITY + 2 that is a trap: once the work stops fitting in the
 * period, every iteration finds the deadline in the past, nothing ever sleeps,
 * and the highest-priority thread in the firmware spins forever -- serial,
 * telemetry and the idle task all stop. The board looks dead.
 *
 * So do not try to make up lost ticks. Resync to now and take the rate hit: the
 * step rate degrades on a board too slow for STEPPER_TICK_MS, which is visible
 * and recoverable, instead of the whole firmware locking up.
 */
static inline void paceTick(eris_systime_t * next){
  eris_systime_t now = eris_get_time();
  if ((int32_t)(*next - now) <= 0){
    *next = now + ERIS_MS_TO_TICKS(STEPPER_TICK_MS);
  }
  eris_sleep_until(next);
}

// Emit n step pulses back to back.
static inline void emitSteps(long n){
  for (long i = 0; i < n; i++){
    digitalWrite(PIN_TMC_STEP, HIGH);
    delayMicroseconds(STEPPER_STEP_PULSE_US);
    digitalWrite(PIN_TMC_STEP, LOW);
    delayMicroseconds(STEPPER_STEP_PULSE_US);
  }
}

/*
 * The motion thread.
 *
 * A stepper wants step edges at microsecond precision; an RTOS thread wakes on
 * a millisecond tick. The usual answer is a hardware timer ISR, but that is a
 * different peripheral on every board Eris supports (IntervalTimer, TimerTc3,
 * the ESP32 timer group) and it is what made the old ErisMPU board-specific.
 *
 * So: keep the RTOS in charge of WHEN, and give up only on the spacing WITHIN a
 * tick. Each tick this thread computes how many steps the current velocity owes
 * it and emits them as one burst, carrying the fractional remainder into the
 * next tick so the average rate is exact. The motor sees pulses bunched at the
 * top of each millisecond instead of evenly spread -- at 16 microsteps that is
 * well inside what the rotor filters out, and the driver's own multistep_filt
 * smooths it further.
 *
 * What this costs: the top rate is STEPPER_MAX_STEPS_PER_TICK per tick, and the
 * burst holds the CPU for that many pulse pairs. Both are bounded and both are
 * in configuration.h. What it buys: the same source runs on every Eris board.
 *
 * The velocity itself follows a trapezoidal ramp -- accelerate at
 * STEPPER_ACCEL, cruise at the feedrate, and start decelerating once the
 * remaining distance is down to v^2/2a. Commanding a stepper straight to speed
 * from standstill does not move it faster, it just stalls.
 */
ERIS_THREAD_WA(waMotion_T, ERIS_STACK_MEDIUM);
ERIS_THREAD_FUNC(Motion_T) {
  (void)arg;
  eris_systime_t next = eris_get_time();

  while (1){
    next += ERIS_MS_TO_TICKS(STEPPER_TICK_MS);

    long target;
    ERIS_CRITICAL_ENTER();
    target = s_target;
    ERIS_CRITICAL_EXIT();

    long remaining = target - s_position;
    int8_t want = (remaining > 0) ? 1 : (remaining < 0) ? -1 : 0;

    /*
     * "Bleed" covers the two cases where the current velocity is pointed the
     * wrong way for the target we now have: the target was reached or revoked
     * while still moving (want == 0), and a new target on the other side
     * (want != s_dir). Both are resolved the same way -- shed speed along the
     * current direction until we are slow enough to stop, then act on the new
     * target. Slamming DIR over at speed would just lose steps, and the
     * position we report would quietly stop matching the machine.
     */
    bool bleed = (want == 0) || (want != s_dir);

    if (bleed && s_velocity <= MIN_VELOCITY){
      s_velocity = 0.0f;
      s_frac     = 0.0f;
      if (want != 0 && want != s_dir){ s_dir = want; applyDir(s_dir); }
      paceTick(&next);
      continue;                        // this tick is the direction change
    }

    // The driver is released: the motor can be back-driven, so stepping it
    // would only make the reported position a lie.
    if (!s_enabled){
      s_velocity = 0.0f;
      s_frac     = 0.0f;
      paceTick(&next);
      continue;
    }

#if STEPPER_STALL_DETECT && TMC_USE_DIAG
    /*
     * The fast path for hard-stop detection. The driver raises DIAG the moment
     * SG_RESULT crosses the threshold, so a digitalRead here catches it within
     * one tick and costs nothing -- no UART transaction on the motion thread,
     * which could not afford one anyway. Without DIAG wired, the same
     * comparison is made by the telemetry thread polling SG_RESULT.
     */
    if (!s_stall && digitalRead(PIN_TMC_DIAG) == HIGH && stallArmed()){
      notifyStall();
      paceTick(&next);
      continue;
    }
#endif

    long  dist     = (remaining >= 0) ? remaining : -remaining;
    float stopDist = (s_velocity * s_velocity) / (2.0f * ACCEL_STEPS);

    if (bleed || (float)dist <= stopDist){
      s_velocity -= ACCEL_STEPS * TICK_S;
      if (s_velocity < MIN_VELOCITY) s_velocity = MIN_VELOCITY;
    }
    else {
      s_velocity += ACCEL_STEPS * TICK_S;
      if (s_velocity > s_feedSteps) s_velocity = s_feedSteps;
    }

    s_frac += s_velocity * TICK_S;
    long n = (long)s_frac;
    if (n > 0){
      s_frac -= (float)n;
      if (!bleed && n > dist){ n = dist; s_frac = 0.0f; }   // never overshoot
      if (n > STEPPER_MAX_STEPS_PER_TICK) n = STEPPER_MAX_STEPS_PER_TICK;
      emitSteps(n);
      s_position += (long)s_dir * n;
    }

    paceTick(&next);
  }
}

// --- API -------------------------------------------------------------------
bool moveTo(float mm){
  if (!s_enabled){
    // Accepting the move would leave the axis "moving" toward a target it
    // cannot reach, and it would then lurch there the moment EN goes back on.
    Error::RaiseError(Error::COMMAND, (char *)"motor is released -- EN 1 to energize");
    return false;
  }
#if STEPPER_SOFT_LIMITS
  if (mm < STEPPER_MIN_POS_MM || mm > STEPPER_MAX_POS_MM){
    // Floats are printed through Print, not sprintf: several of the cores Eris
    // targets link newlib-nano, whose printf drops %f and silently prints
    // nothing at all.
    Error::RaiseError(Error::COMMAND, (char *)"G0 target outside soft limits");
    eriscommon::print("  target ");      eriscommon::print((double)mm, 3);
    eriscommon::print(" mm, limits ");   eriscommon::print((double)STEPPER_MIN_POS_MM, 1);
    eriscommon::print(" .. ");           eriscommon::println((double)STEPPER_MAX_POS_MM, 1);
    return false;
  }
#endif
  long steps = (long)(toSteps(mm) + (mm >= 0 ? 0.5f : -0.5f));
  ERIS_CRITICAL_ENTER();
  s_target = steps;
  ERIS_CRITICAL_EXIT();

  // A new move is the acknowledgement of the last stall -- and it is usually
  // the move that backs the axis off whatever it hit, so refusing to run while
  // the latch is set would leave the machine stuck against it.
  if (s_stall){
    clearStall();
    eriscommon::println("stall latch cleared by new move");
  }
  // Restarts the arming window: the ramp at the start of THIS move must not be
  // read as a stall.
  s_moveStartMs = millis();
  return true;
}

float setFeed(float mmPerMin){
  if (mmPerMin < STEPPER_MIN_FEED) mmPerMin = STEPPER_MIN_FEED;
  if (mmPerMin > STEPPER_MAX_FEED) mmPerMin = STEPPER_MAX_FEED;
  float steps = mmPerMin * STEPPER_STEPS_PER_MM / 60.0f;
  // The burst scheme cannot emit more than STEPPER_MAX_STEPS_PER_TICK per tick,
  // so a feedrate above that would silently not be reached. Clamp it here,
  // where the number is still visible to the caller.
  if (steps > MAX_VELOCITY) steps = MAX_VELOCITY;
  s_feedSteps = steps;
  return toMm(steps) * 60.0f;
}

float feed(void){ return toMm(s_feedSteps) * 60.0f; }

void stop(void){
  /*
   * Aim at where a proper deceleration would put us, rather than at the
   * current position: setting target == position would demand an instant halt
   * from full speed, which a stepper answers by losing steps.
   */
  ERIS_CRITICAL_ENTER();
  float v        = s_velocity;
  long  pos      = s_position;
  int8_t dir     = s_dir;
  ERIS_CRITICAL_EXIT();

  long stopDist = (long)((v * v) / (2.0f * ACCEL_STEPS) + 0.5f);
  ERIS_CRITICAL_ENTER();
  s_target = pos + (long)dir * stopDist;
  ERIS_CRITICAL_EXIT();
}

void zero(void){
  ERIS_CRITICAL_ENTER();
  s_target   = 0;
  s_position = 0;
  ERIS_CRITICAL_EXIT();
  s_frac = 0.0f;
  clearStall();   // "this is where we are now" -- the old latch is stale
}

void enable(bool on){
  // Active LOW on every TMC2209 breakout.
  digitalWrite(PIN_TMC_EN, on ? LOW : HIGH);
  if (!on){
    // A released motor can be turned by hand, so the step count stops
    // describing the machine. Drop the pending move with it rather than
    // resuming toward a target the axis may no longer be anywhere near.
    ERIS_CRITICAL_ENTER();
    s_target = s_position;
    ERIS_CRITICAL_EXIT();
  }
  else {
    clearStall();          // re-energizing is an acknowledgement
    s_moveStartMs = millis();
  }
  s_enabled = on;
}

float positionMm(void){
  ERIS_CRITICAL_ENTER();
  long p = s_position;
  ERIS_CRITICAL_EXIT();
  return toMm((float)p);
}

float targetMm(void){
  ERIS_CRITICAL_ENTER();
  long t = s_target;
  ERIS_CRITICAL_EXIT();
  return toMm((float)t);
}

float velocityMmS(void){
  ERIS_CRITICAL_ENTER();
  float v  = s_velocity;
  int8_t d = s_dir;
  ERIS_CRITICAL_EXIT();
  return toMm(v) * (float)d;
}

bool isMoving(void){
  ERIS_CRITICAL_ENTER();
  bool m = (s_position != s_target) || (s_velocity > 0.0f);
  ERIS_CRITICAL_EXIT();
  return m;
}

bool isEnabled(void){ return s_enabled; }

// --- Hard-stop detection ---------------------------------------------------
bool stallArmed(void){
#if STEPPER_STALL_DETECT
  if (!s_enabled || s_stall)                    return false;
  if (s_velocity < STALL_MIN_VELOCITY)          return false;
  // SG_RESULT is still settling while the load angle changes, so the whole
  // acceleration ramp reads like a stall. Wait it out.
  if ((uint32_t)(millis() - s_moveStartMs) < STEPPER_STALL_ARM_MS) return false;
  return true;
#else
  return false;
#endif
}

void notifyStall(void){
  if (s_stall) return;
  s_stall         = true;
  s_stallReported = false;   // the telemetry thread prints it; see above
#if STEPPER_STALL_HALT
  /*
   * Stop dead rather than decelerating. The usual reason to ramp down is to
   * avoid losing steps -- but the axis is already against something and the
   * steps are already lost, so the only thing a ramp would add is more
   * distance travelled into whatever we hit.
   */
  ERIS_CRITICAL_ENTER();
  s_target   = s_position;
  s_velocity = 0.0f;
  ERIS_CRITICAL_EXIT();
  s_frac = 0.0f;
#endif
}

bool stalled(void){ return s_stall; }

void clearStall(void){
  s_stall         = false;
  s_stallReported = true;
}

// Called from the telemetry thread once per sample: it owns the reporting, and
// (when DIAG is not wired) the SG_RESULT polling that detects the stall in the
// first place. Returns the SG value to put in the sample.
uint16_t stallService(void){
  uint16_t sg = TMC::lastStallGuard();

#if STEPPER_STALL_DETECT
  if (!TMC::present()) return sg;

  static uint8_t divider = 0;
  if (++divider >= STEPPER_SG_DIVIDER){
    divider = 0;
    uint16_t v;
    // A failed read leaves v alone and is simply skipped -- never treated as a
    // low reading, which is what a stall looks like.
    if (TMC::readStallGuard(v)){
      sg = v;
#if !TMC_USE_DIAG
      if (stallArmed() && sg <= (uint16_t)(2 * TMC::stallThreshold())){
        notifyStall();
      }
#endif
    }
  }

  if (!s_stallReported){
    s_stallReported = true;
    // Distinguish a genuine mechanical stall from a driver fault: DIAG goes
    // high for both, and "hit a hard stop" vs "thermal shutdown" want very
    // different responses from whoever is reading this.
    uint16_t st = TMC::readStatus();
    if (st & (TMC::ST_OT | TMC::ST_OTPW | TMC::ST_S2GA | TMC::ST_S2GB |
              TMC::ST_S2VSA | TMC::ST_S2VSB)){
      Error::RaiseError(Error::SENSOR,
        (char *)"driver fault during move (see TMC), motion halted");
    }
    else {
      Error::RaiseError(Error::SENSOR, (char *)"hard stop detected, motion halted");
    }
    eriscommon::print("  stalled at ");  eriscommon::print((double)positionMm(), 3);
    eriscommon::print(" mm, sg ");       eriscommon::print((double)sg, 0);
    eriscommon::print(" <= ");           eriscommon::print((double)(2 * TMC::stallThreshold()), 0);
    eriscommon::println(" -- next G0 clears the latch");
  }
#endif
  return sg;
}

void begin(void){
  pinMode(PIN_TMC_STEP, OUTPUT);
  pinMode(PIN_TMC_DIR,  OUTPUT);
  pinMode(PIN_TMC_EN,   OUTPUT);
#if STEPPER_STALL_DETECT && TMC_USE_DIAG
  // The TMC2209 drives DIAG push-pull, so no pull-up. It idles low.
  pinMode(PIN_TMC_DIAG, INPUT);
#endif
  digitalWrite(PIN_TMC_STEP, LOW);
  applyDir(s_dir);
  enable(true);

  setFeed(STEPPER_DEFAULT_FEED);

  motionThread = eris_thread_create(waMotion_T, ERIS_STACK_MEDIUM,
                                    ERIS_NORMAL_PRIORITY + 2, Motion_T, NULL);

  eriscommon::print("Motion ready: ");
  eriscommon::print((double)STEPPER_STEPS_PER_MM, 1); eriscommon::print(" steps/mm, feed ");
  eriscommon::print((double)feed(), 0);               eriscommon::print(" mm/min, accel ");
  eriscommon::print((double)STEPPER_ACCEL, 0);        eriscommon::println(" mm/s2");
}

} // namespace Motion

// ---------------------------------------------------------------------------
// Axis telemetry
//
// Generates Stepper::buffer, Stepper::start() and the sampling thread. There is
// no device to talk to for most of it -- the interesting state already lives in
// this file -- but going through ERIS_SENSOR means the buffer, the timestamp
// and the thread come out the same shape as every other Eris signal, and
// streaming/SD/ASCII all work with no extra code.
// ---------------------------------------------------------------------------
// ERIS_SENSOR_EX rather than ERIS_SENSOR: this thread does the stall reporting,
// which goes through the error handler and the packet layer, and that does not
// fit in the default MEDIUM stack tier.
ERIS_SENSOR_EX(Stepper, StepperSample_t, STEPPER_FREQUENCY_HZ,
               ERIS_STACK_LARGE, ERIS_NORMAL_PRIORITY + 1) {
  s.position = Motion::positionMm();
  s.target   = Motion::targetMm();
  s.velocity = Motion::velocityMmS();
  s.moving   = Motion::isMoving()  ? 1 : 0;
  s.enabled  = Motion::isEnabled() ? 1 : 0;
  s.pad      = 0;

  // Polls SG_RESULT (and detects the stall, when DIAG is not wired), then
  // reports a latched one. Owning that here keeps the UART and the printing
  // off the 1 ms motion thread.
  s.sg      = Motion::stallService();
  s.stalled = Motion::stalled() ? 1 : 0;

  // DRV_STATUS costs a UART round trip, which is far too expensive to do on
  // every telemetry tick. Poll it every STEPPER_STATUS_DIVIDER samples and
  // carry the cached value in between; a thermal fault does not need 100 Hz.
  static uint8_t divider = 0;
  if (++divider >= STEPPER_STATUS_DIVIDER){
    divider = 0;
    s.status = TMC::present() ? TMC::readStatus() : 0;
  }
  else {
    s.status = TMC::lastStatus();
  }
  return true;
}
