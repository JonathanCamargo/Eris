#include "Eris.h"
#include "servos.h"

#include <Servo.h>

namespace Servos{

  static Servo servos[NUM_SERVOS];
  static const uint8_t pins[NUM_SERVOS] = {SERVO_PINS};

  // Derived constants from configuration
  static constexpr float maxStep = SERVO_SMOOTH_SPEED * SERVO_SMOOTH_LOOP_MS / 1000.0f;
  static constexpr float gain    = maxStep / SERVO_SMOOTH_DECEL_DEG;
  static constexpr float epsilon = 0.02f;

  // Smooth movement state
  static float currentAngles[NUM_SERVOS];
  static float targetAngles[NUM_SERVOS];
  static eris_thread_ref_t smoothThread = NULL;

  static float stepToward(float current, float target){
    float diff = target - current;
    if (diff > -epsilon && diff < epsilon) return target;
    float step = diff * gain;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    return current + step;
  }

  static void applyAngle(uint8_t channel, float angle){
    servos[channel].write((int)(angle + 0.5f));
  }

  // Thread that steps servos toward their targets
  ERIS_THREAD_WA(waSmoothServo_T, ERIS_STACK_MEDIUM);
  ERIS_THREAD_FUNC(SmoothServo_T) {
    while(1){
      for (uint8_t i = 0; i < NUM_SERVOS; i++){
        float diff = targetAngles[i] - currentAngles[i];
        if (diff < -epsilon || diff > epsilon){
          currentAngles[i] = stepToward(currentAngles[i], targetAngles[i]);
          applyAngle(i, currentAngles[i]);
        }
      }
      eris_sleep_ms(SERVO_SMOOTH_LOOP_MS);
    }
  }

  void start(){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      servos[i].attach(pins[i]);
      currentAngles[i] = 90.0f;
      targetAngles[i]  = 90.0f;
      servos[i].write(90);
    }
    smoothThread = eris_thread_create(waSmoothServo_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+1, SmoothServo_T, NULL);
    Serial.println("Servos ready");
  }

  void move(uint8_t channel, float angle){
    if (channel >= NUM_SERVOS) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    // Immediate move — also update smooth state so the thread doesn't fight us
    currentAngles[channel] = angle;
    targetAngles[channel] = angle;
    applyAngle(channel, angle);
  }

  void moveAll(float* angles){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      move(i, angles[i]);
    }
  }

  void smoothMove(uint8_t channel, float angle){
    if (channel >= NUM_SERVOS) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    targetAngles[channel] = angle;
  }

  void smoothMoveAll(float* angles){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      smoothMove(i, angles[i]);
    }
  }

}
