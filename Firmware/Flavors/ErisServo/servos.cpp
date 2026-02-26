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
    if (diff > SERVO_SMOOTH_MAX_STEP) return current + SERVO_SMOOTH_MAX_STEP;
    if (diff < -SERVO_SMOOTH_MAX_STEP) return current - SERVO_SMOOTH_MAX_STEP;
    return target;
  }

  static void applyAngle(uint8_t channel, float angle){
    int pulse = map((int)angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
  }

  // Thread that steps servos toward their targets
  ERIS_THREAD_WA(waSmoothServo_T, ERIS_STACK_MEDIUM);
  ERIS_THREAD_FUNC(SmoothServo_T) {
    while(1){
      for (uint8_t i = 0; i < NUM_SERVOS; i++){
        if (currentAngles[i] != targetAngles[i]){
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

  void move(uint8_t channel, int angle){
    if (channel >= NUM_SERVOS) return;
    // Clip angle to valid range
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    // Immediate move — also update smooth state so thread doesn't fight
    currentAngles[channel] = angle;
    targetAngles[channel] = angle;
    int pulse = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
  }

  void moveAll(int* angles){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      move(i, angles[i]);
    }
  }

  void smoothMove(uint8_t channel, int angle){
    if (channel >= NUM_SERVOS) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    targetAngles[channel] = angle;
  }

  void smoothMoveAll(int* angles){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      smoothMove(i, angles[i]);
    }
  }

}
