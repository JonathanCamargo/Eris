/* Test code for ADS1256_teensy_lib library. This code creates an ADS1256 object
defined by the libary, turn it to collecting mode and run continuously displaying
data via serial. */

#include <SPI.h>
#include "ADS1256_4CH.h"

#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 28

// Create the object and the function to hold the interrupt. 
ADS1256 sensor1(PIN_ADC0_SS,PIN_ADC0_DRDY);




// Setup an interrupt on DRDY pin to collect the data and check if the buffer is full to get the last bufferSize() readings.
volatile bool bufferFlag=false; // Use as a flag to enable buffer reading from the main loop.
bool wasFull=false; // Use as a flag to determine if there is buffer overflow.
void interruptHandler1(void) {  
  if (sensor1.collectData()){    
    bufferFlag=true;    
  } 
}


void setup() {
  // Begin serial connection and wait until it is activated
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial);
     
  // Turn the ADS1256 into Continuous read mode
  sensor1.startCollecting();
  
  // Attach the interrupt tot he DRDY pin defined by the class
  attachInterrupt(digitalPinToInterrupt(PIN_ADC0_DRDY),interruptHandler1,FALLING);


}


int count = 0;
uint16_t buffersize=sensor1.bufferSize();

void loop() {

  delay(5);


  /*
  if(Serial1.available()) {
  while (Serial1.available()) {
    char incoming = Serial1.read();
    Serial.print(incoming);
    //Serial.print('\t');
  }
  Serial.println();
  }
  */

  //Serial.println("hola");
  if (bufferFlag==true){
    //Get data and print
    float * data_ch0=sensor1.get_data(0);
    //float * data_ch1=sensor1.get_data(1);
    //float * data_ch2=sensor1.get_data(2);
    //float * data_ch3=sensor1.get_data(3);

/*
      Serial.print(analogRead(19)/(1023.0) * 3.3);      //Transfer data
      Serial.print('\t');
      Serial.print(analogRead(18)/(1023.0) * 3.3);      //Transfer data
      Serial.print('\t');
      Serial.print(analogRead(17)/(1023.0) * 3.3);      //Transfer data
      Serial.print('\t');
      Serial.println(analogRead(16)/(1023.0) * 3.3);      //Transfer data
/*
      Serial.print(data_ch0[0],4);      //Transfer data
      Serial.print('\t');
      Serial.print(data_ch1[0],4);      //Transfer data
      Serial.print('\t');
      Serial.print(data_ch2[0],4);      //Transfer data
      Serial.print('\t');
      Serial.println(data_ch3[0],4);      //Transfer data
//*/    
   for (uint16_t i=0;i<buffersize;i++){

      
      Serial.println(data_ch0[i],4);      //Transfer data
      //Serial.print('\t');
      //Serial.print(data_ch1[i],4);      //Transfer data
      //Serial.print('\t');
      //Serial.print(data_ch2[i],4);      //Transfer data
      //Serial.print('\t');
      //Serial.println(data_ch3[i],4);      //Transfer data
      
    }
    bufferFlag=false;      
    if (wasFull==true){
      Serial.println("Error buffer overflow");
    }
    wasFull=true;    
  }
  else{
    wasFull=false;
  }
  
  /*
  if (wasFull==true){    
  Serial.println("Register report:");
  Serial.println("====================================");  
  uint8_t id=sensor1.ADS1256_ReadReg(0x00);
  Serial.print("STATUS:");Serial.println(id,HEX);
  id=sensor1.ADS1256_ReadReg(0x01);
  Serial.print("MUX:");Serial.println(id,HEX);
  id=sensor1.ADS1256_ReadReg(0x02);
  Serial.print("ADCON:");Serial.println(id,HEX);
  id=sensor1.ADS1256_ReadReg(0x03);
  Serial.print("DRATE:");Serial.println(id,HEX);
  id=sensor1.ADS1256_ReadReg(0x04);
  Serial.print("GPIO:");Serial.println(id,HEX);     
  }
  */
  
  
}


