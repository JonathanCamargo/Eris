#include <yostimu.h>
#include <SPI.h>

//#include <DmaSpi.h>

YostIMU imu0(14);

uint8_t spibuffer[128];
static float x[9]={0};

void setup() {
  SPI.begin();
  // put your setup code here, to run once:
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);  
  pinMode(8,OUTPUT);
  delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:
  
  long t1=micros();  
  imu0.WriteCommand(GET_ALL);   
  delayMicroseconds(10);
  imu0.Read(spibuffer,36);   
  long t2=micros();
  //Serial.println(t2-t1);
  while(!imu0.DataReady()){  
    digitalWrite(8,!digitalRead(8)); //Do something while waiting :)
    delayMicroseconds(2);  
  }
  imu0.ReadFloats((uint8_t *)spibuffer,(float*)x,9);
  
  
  // Gyro
  Serial.print(x[0],4);
  Serial.print(',');
  Serial.print(x[1],4);
  Serial.print(',');
  Serial.print(x[2],4);
  Serial.println(' ');
  Serial.send_now();
  
  
  delay(10);
}
