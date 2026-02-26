#include "Eris.h"
#include "servos.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace Servos{

  Adafruit_PWMServoDriver pwm(PCA9685_I2C_ADDR);

  // Smooth movement state
  static float currentAngles[NUM_SERVOS];
  static float targetAngles[NUM_SERVOS];
  static eris_thread_ref_t smoothThread = NULL;

  static float stepToward(float current, float target){
    float diff = target - current;
    // Snap to target when close enough (avoids float oscillation)
    if (diff > -SERVO_SMOOTH_EPSILON && diff < SERVO_SMOOTH_EPSILON) return target;
    // Ease-out: proportional step, clamped to max speed
    float step = diff * SERVO_SMOOTH_GAIN;
    if (step > SERVO_SMOOTH_MAX_STEP) step = SERVO_SMOOTH_MAX_STEP;
    if (step < -SERVO_SMOOTH_MAX_STEP) step = -SERVO_SMOOTH_MAX_STEP;
    return current + step;
  }

  static void applyAngle(uint8_t channel, float angle){
    // Float math for full 12-bit PCA9685 resolution (~4096 steps)
    float pulse = SERVO_MIN_PULSE + (angle - SERVO_MIN_ANGLE) * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    pwm.setPWM(channel, 0, (uint16_t)(pulse + 0.5f));
  }

  // Thread that steps servos toward their targets
  ERIS_THREAD_WA(waSmoothServo_T, ERIS_STACK_MEDIUM);
  ERIS_THREAD_FUNC(SmoothServo_T) {
    while(1){
      for (uint8_t i = 0; i < NUM_SERVOS; i++){
        float diff = targetAngles[i] - currentAngles[i];
        if (diff < -SERVO_SMOOTH_EPSILON || diff > SERVO_SMOOTH_EPSILON){
          currentAngles[i] = stepToward(currentAngles[i], targetAngles[i]);
          applyAngle(i, currentAngles[i]);
        }
      }
      eris_sleep_ms(SERVO_SMOOTH_LOOP_MS);
    }
  }

  void start(){
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(50); // 50Hz for standard servos
    // Initialize smooth state to center position
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      currentAngles[i] = 90.0;
      targetAngles[i] = 90.0;
    }
    // Start smooth movement thread
    smoothThread = eris_thread_create(waSmoothServo_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+1, SmoothServo_T, NULL);
    Serial.println("Servos ready");
  }

  void move(uint8_t channel, float angle){
    if (channel >= NUM_SERVOS) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    // Immediate move — also update smooth state so thread doesn't fight
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
