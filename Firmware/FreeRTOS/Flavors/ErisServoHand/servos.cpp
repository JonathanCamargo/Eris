#include "Eris.h"

namespace Servos{
  PCA9685_ServoEval pwmServo;
  PCA9685 pwmController(WIRE_PORT);           // Library using Wire1 @400kHz, and default B000000 (A5-A0) i2c address

  void start(){
     WIRE_PORT.begin();
     pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line
     pwmController.init();               // Initializes module using default totem-pole driver mode, and default disabled phase balancer
     pwmController.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length
  }

  void move(int* x){
    // Thumb is inverted
    x[0]=90-x[0];
    for (uint8_t i=0; i<5; i++){
      // clip to min max angles:
      x[i]= x[i]>MAX_ANGLE ? MAX_ANGLE : x[i];
      x[i]= x[i]<MIN_ANGLE ? MIN_ANGLE: x[i];      
      pwmController.setChannelPWM(i, pwmServo.pwmForAngle(x[i]));
    }    
  }

}
