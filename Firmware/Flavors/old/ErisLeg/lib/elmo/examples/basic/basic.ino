#include <elmo.h>

#define ELMO_CANID1 127
#define ELMO_CANID2 126

Elmo motor1(ELMO_CANID1);
Elmo motor2(ELMO_CANID2);

int delayTime = 1000;
int motor1Pos,motor1Vel = 0;
int motor2Pos,motor2Vel = 0;
//int motor1PosAux = 0;

int count=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Before sent first message");
  delay(delayTime);
  
  Can0.begin(1000000);
  Serial.println("Initialized CAN");
  delay(delayTime);

  ///////////////////////////////////////////////////
  // MOTOR 2 TEST

  motor2.connect();
  motor1.connect();

  
  
  Serial.println("\nConnected Motor 2");
  delay(delayTime);

  motor2.motor_on();
  Serial.println("\nMotor 2 on");
  delay(delayTime);
  motor2.get_position(&motor2Pos);
  Serial.print("\nMotor 2 Initial Position: ");
  Serial.println(motor2Pos);

  Serial.println("Turning on Motor 2");
  motor2.set_current(0.5);
  delay(delayTime);

  Serial.println("Turning off Motor 2");
  motor2.set_current(0);
  delayMicroseconds(10);

  motor2.get_position(&motor2Pos);
  Serial.print("\nMotor 2 Final Position: ");
  Serial.println(motor2Pos);

  ///////////////////////////////////////////////////
  // MOTOR 1 TEST
  Serial.println("\nConnected Motor 1");
  delay(delayTime);

  motor1.motor_on();
  Serial.println("\nMotor 1 on");
  delay(delayTime);
  
  motor1.get_position(&motor1Pos);
  Serial.print("\nMotor 1 Initial Position: ");
  Serial.print(motor1Pos);
/*
  motor1.get_aux_position(&motor1PosAux);
  Serial.print("  Motor 1 Aux Position: ");
  Serial.println(motor1PosAux);
  */

  Serial.println("Turning on Motor 1");
  motor1.set_current(0.5);
  delay(delayTime);

  Serial.println("Turning off Motor 1");
  motor1.set_current(0);

  motor1.get_position(&motor1Pos);
  Serial.print("\nMotor 1 Final Position: ");
  Serial.println(motor1Pos);

  Serial.println("END OF TEST");

}

void loop() {

  motor1.set_current(0);
  motor2.set_current(0);
  
  if (count==0) {
    motor2.set_current(0.5);
    delay(3000);
    motor2.set_current(0);
    motor1.set_current(0.5);
    delay(3000);
    motor1.set_current(0);
    count++;
  }
  
  if (motor1.get_position(&motor1Pos)){
  Serial.print("Motor 1 Position: ");
  Serial.println(motor1Pos);
  }
  else{
    Serial.println("Error pos motor1");
  }
  if (motor2.get_position(&motor2Pos)){
    Serial.print("Motor 2 Position: ");
    Serial.println(motor2Pos);
  }
  else{
    Serial.println("Error pos motor2");
  }

if (motor1.get_velocity(&motor1Vel)){
  Serial.print("Motor 1 Velocity: ");
  Serial.println(motor1Vel);
  }
  else{
    Serial.println("Error pos motor1");
  }
  if (motor2.get_velocity(&motor2Vel)){
    Serial.print("Motor 2 Velocity: ");
    Serial.println(motor2Vel);
  }
  else{
    Serial.println("Error pos motor2");
  }
  
  //motor1.get_aux_position(&motor1PosAux);
  //Serial.print("  Motor 1 Aux Position: ");
  //Serial.println(motor1PosAux);
  delay(delayTime);
}
