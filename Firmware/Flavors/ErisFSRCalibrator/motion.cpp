#include "Eris.h"
#include "motion.h"

// Catch the ESP32-only pin mistakes at compile time (no-ops elsewhere).
ERIS_PIN_ASSERT_OUTPUT(PIN_A4988_STEP);
ERIS_PIN_ASSERT_OUTPUT(PIN_A4988_DIR);
ERIS_PIN_ASSERT_OUTPUT(PIN_A4988_EN);

static_assert(CAL_PRESS_DIR == 1 || CAL_PRESS_DIR == -1,
              "CAL_PRESS_DIR must be +1 or -1");

namespace Motion{

// --- Units -----------------------------------------------------------------
// Below the API boundary everything is steps and steps/s: integer steps are
// what the hardware does, and float millimetres drift over many moves.
static const float TICK_S       = STEPPER_TICK_MS / 1000.0f;
static const float ACCEL_STEPS  = STEPPER_ACCEL * STEPPER_STEPS_PER_MM;      // steps/s^2
static const float MIN_VELOCITY = STEPPER_MIN_FEED * STEPPER_STEPS_PER_MM / 60.0f;
static const float MAX_VELOCITY = (float)STEPPER_MAX_STEPS_PER_TICK / TICK_S;

static inline float toSteps(float mm){ return mm * STEPPER_STEPS_PER_MM; }
static inline float toMm(float steps){ return steps / STEPPER_STEPS_PER_MM; }

// --- State -----------------------------------------------------------------
// s_position is written only by the motion thread (and halt/zero, inside a
// critical section); s_target by commands. A long is not written atomically on
// every Eris board, so readers take a critical section too.
static volatile long   s_position = 0;      // steps
static volatile long   s_target   = 0;      // steps
static volatile float  s_velocity = 0.0f;   // steps/s, magnitude only
static volatile int8_t s_dir      = 1;      // direction latched on the DIR pin
static volatile bool   s_enabled  = false;
static volatile bool   s_pressBlocked = false;

static float s_feedSteps = STEPPER_DEFAULT_FEED * STEPPER_STEPS_PER_MM / 60.0f;
static float s_frac      = 0.0f;            // sub-step carried between ticks

static eris_thread_ref_t motionThread = NULL;

static inline void applyDir(int8_t dir){
  bool level = (dir > 0);
#if STEPPER_INVERT_DIR
  level = !level;
#endif
  digitalWrite(PIN_A4988_DIR, level ? HIGH : LOW);
  // A4988 wants 200 ns DIR setup before the next STEP edge.
  delayMicroseconds(1);
}

/*
 * Sleep to the next tick boundary, resyncing if this tick overran.
 * eris_sleep_until() returns at once for a deadline in the past; at
 * ERIS_NORMAL_PRIORITY + 2 that would spin forever once the work stops fitting
 * the period, and starve serial and streaming. Take the rate hit instead.
 */
static inline void paceTick(eris_systime_t * next){
  eris_systime_t now = eris_get_time();
  if ((int32_t)(*next - now) <= 0){
    *next = now + ERIS_MS_TO_TICKS(STEPPER_TICK_MS);
  }
  eris_sleep_until(next);
}

static inline void emitSteps(long n){
  for (long i = 0; i < n; i++){
    digitalWrite(PIN_A4988_STEP, HIGH);
    delayMicroseconds(STEPPER_STEP_PULSE_US);
    digitalWrite(PIN_A4988_STEP, LOW);
    delayMicroseconds(STEPPER_STEP_PULSE_US);
  }
}

/*
 * The motion thread. Same scheme as ErisStepper: the RTOS decides WHEN (a 1 ms
 * tick), and each tick emits the steps the current velocity owes as one burst,
 * carrying the fractional remainder so the average rate is exact. No hardware
 * timer, so the same source runs on every Eris board.
 *
 * Velocity follows a trapezoidal ramp: accelerate at STEPPER_ACCEL, cruise at
 * the feedrate, decelerate once the remaining distance is down to v^2/2a.
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

    // Velocity pointed the wrong way for the current target (reached, revoked,
    // or reversed): shed speed along the current direction until slow enough
    // to stop, then act on the new target. Flipping DIR at speed loses steps.
    bool bleed = (want == 0) || (want != s_dir);

    if (bleed && s_velocity <= MIN_VELOCITY){
      s_velocity = 0.0f;
      s_frac     = 0.0f;
      if (want != 0 && want != s_dir){ s_dir = want; applyDir(s_dir); }
      paceTick(&next);
      continue;
    }

    // Released driver: the motor can be back-driven, so stepping would only
    // make the reported position a lie.
    if (!s_enabled){
      s_velocity = 0.0f;
      s_frac     = 0.0f;
      paceTick(&next);
      continue;
    }

    // Belt and braces for the press guard: a move already in flight toward the
    // load is cut here too, not only when the next G0 arrives.
    if (s_pressBlocked && s_dir == CAL_PRESS_DIR && !bleed){
      halt();
      paceTick(&next);
      continue;
    }

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
      ERIS_CRITICAL_ENTER();
      s_position += (long)s_dir * n;
      ERIS_CRITICAL_EXIT();
    }

    paceTick(&next);
  }
}

// --- API -------------------------------------------------------------------
bool moveTo(float mm){
  if (!s_enabled){
    Error::RaiseError(Error::COMMAND, (char *)"motor is released -- EN 1 to energize");
    return false;
  }
#if STEPPER_SOFT_LIMITS
  if (mm < STEPPER_MIN_POS_MM || mm > STEPPER_MAX_POS_MM){
    // Floats go through Print, not sprintf: newlib-nano's printf drops %f.
    Error::RaiseError(Error::COMMAND, (char *)"G0 target outside soft limits");
    eriscommon::print("  target ");      eriscommon::print((double)mm, 3);
    eriscommon::print(" mm, limits ");   eriscommon::print((double)STEPPER_MIN_POS_MM, 1);
    eriscommon::print(" .. ");           eriscommon::println((double)STEPPER_MAX_POS_MM, 1);
    return false;
  }
#endif
  long steps = (long)(toSteps(mm) + (mm >= 0 ? 0.5f : -0.5f));

  if (s_pressBlocked){
    ERIS_CRITICAL_ENTER();
    long pos = s_position;
    ERIS_CRITICAL_EXIT();
    if ((steps - pos) * (long)CAL_PRESS_DIR > 0){
      Error::RaiseError(Error::COMMAND,
        (char *)"overload guard: only moves that back off the load are accepted");
      return false;
    }
  }

  ERIS_CRITICAL_ENTER();
  s_target = steps;
  ERIS_CRITICAL_EXIT();
  return true;
}

float setFeed(float mmPerMin){
  if (mmPerMin < STEPPER_MIN_FEED) mmPerMin = STEPPER_MIN_FEED;
  if (mmPerMin > STEPPER_MAX_FEED) mmPerMin = STEPPER_MAX_FEED;
  float steps = mmPerMin * STEPPER_STEPS_PER_MM / 60.0f;
  // The burst scheme tops out at STEPPER_MAX_STEPS_PER_TICK per tick; clamp
  // here, where the caller can still see the number.
  if (steps > MAX_VELOCITY) steps = MAX_VELOCITY;
  s_feedSteps = steps;
  return toMm(steps) * 60.0f;
}

float feed(void){ return toMm(s_feedSteps) * 60.0f; }

void stop(void){
  // Aim where a proper deceleration ends, not at the current position:
  // target == position from speed is an instant halt, which loses steps.
  ERIS_CRITICAL_ENTER();
  float  v   = s_velocity;
  long   pos = s_position;
  int8_t dir = s_dir;
  ERIS_CRITICAL_EXIT();

  long stopDist = (long)((v * v) / (2.0f * ACCEL_STEPS) + 0.5f);
  ERIS_CRITICAL_ENTER();
  s_target = pos + (long)dir * stopDist;
  ERIS_CRITICAL_EXIT();
}

void halt(void){
  // No ramp. At calibration feedrates the axis is slow enough to stop dead
  // without losing steps, and on an overload every extra step is more force
  // into the load cell.
  ERIS_CRITICAL_ENTER();
  s_target   = s_position;
  s_velocity = 0.0f;
  ERIS_CRITICAL_EXIT();
  s_frac = 0.0f;
}

void zero(void){
  ERIS_CRITICAL_ENTER();
  s_target   = 0;
  s_position = 0;
  ERIS_CRITICAL_EXIT();
  s_frac = 0.0f;
}

void enable(bool on){
  // Active LOW on the A4988.
  digitalWrite(PIN_A4988_EN, on ? LOW : HIGH);
  if (!on){
    // A released motor can be turned by hand; drop the pending move rather
    // than resuming toward a target the axis may no longer be near.
    ERIS_CRITICAL_ENTER();
    s_target = s_position;
    ERIS_CRITICAL_EXIT();
  }
  s_enabled = on;
}

void blockPress(bool on){
  s_pressBlocked = on;
  if (on){
    // Cut a press already in progress right now, from the caller's thread,
    // rather than waiting up to a tick for the motion thread to notice.
    ERIS_CRITICAL_ENTER();
    bool pressing = (s_target - s_position) * (long)CAL_PRESS_DIR > 0;
    ERIS_CRITICAL_EXIT();
    if (pressing) halt();
  }
}

bool pressBlocked(void){ return s_pressBlocked; }

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
  float  v = s_velocity;
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

void begin(void){
  pinMode(PIN_A4988_STEP, OUTPUT);
  pinMode(PIN_A4988_DIR,  OUTPUT);
  pinMode(PIN_A4988_EN,   OUTPUT);
  digitalWrite(PIN_A4988_STEP, LOW);
  applyDir(s_dir);
  enable(true);

  setFeed(STEPPER_DEFAULT_FEED);

  motionThread = eris_thread_create(waMotion_T, ERIS_STACK_MEDIUM,
                                    ERIS_NORMAL_PRIORITY + 2, Motion_T, NULL);

  eriscommon::print("Motion ready: ");
  eriscommon::print((double)STEPPER_STEPS_PER_MM, 1); eriscommon::print(" steps/mm (");
  eriscommon::print((double)A4988_MICROSTEPS, 0);     eriscommon::print(" usteps -- check MS1-3), feed ");
  eriscommon::print((double)feed(), 0);               eriscommon::println(" mm/min");
}

} // namespace Motion
