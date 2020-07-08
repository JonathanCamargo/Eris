#include "ADS1299.h"

#include <SPI.h>
#include <SerialCommand.h>

SerialCommand scmd;
#define PIN_SS 8
#define PIN_DRDY 7
ADS1299 ads(PIN_SS,PIN_DRDY);

char strbuffer[128];


uint16_t buffersize=ads.bufferSize();

void test(){ 
  char * arg = scmd.next();  
  if (arg != NULL) {
    uint8_t val;
    sscanf(arg,"%d",&val);
    sprintf(strbuffer,"Reading register %s (%d):",arg,val);    
    Serial.println(strbuffer);
    uint8_t id=ads.ADS1299_ReadReg(val);
    Serial.println(id,HEX);       
  } else {    
  }
}

// Setup an interrupt on DRDY pin to collect the data and check if the buffer is full to get the last bufferSize() readings.
volatile bool bufferFlag=false; // Use as a flag to enable buffer reading from the main loop.
bool wasFull=false; // Use as a flag to determine if there is buffer overflow.
void interruptHandler1(void) {  
  //Serial.println("hola");
  if (ads.collectData()){    
    bufferFlag=true;    
  } 
}


void setup() {
  delay(1500);
  Serial.println("Iniciando");
  scmd.addCommand("R",test);
  /*uint8_t id=ads.ADS1299_ReadReg(0x01);
  Serial.print("MUX:");Serial.println(id,HEX);
  delay(100);
  */
  // Attach the interrupt tot he DRDY pin defined by the class
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY),interruptHandler1,FALLING);
  delay(10);
  ads.startCollecting();
}

void loop() {
  // put your main code here, to run repeatedly:
  scmd.readSerial();
  if (bufferFlag==true){
    //Get data and print
    float * data_ch0=ads.get_data(0);
     
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
  delay(100);
  uint32_t test=0;
  uint8_t aa=0x01;
  test=(aa<<8);

  Serial.println(test,HEX);
  */

}
