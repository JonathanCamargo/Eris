/* Test code for ADS1256_teensy_lib library. This code creates an ADS1256 object
defined by the libary and uses simple read functions to check the register values
 */
#include <SPI.h>
#include "ADS1256_4CH.h"

//Slave select pin
#define PIN_ADC0_SS 6

// Create the object and the static function to hold the interrupt. 
// Interrupts can only be staic functions so there is no way to define interrupt inside the class
ADS1256 sensor1(PIN_ADC0_SS);

// Setup interrupts to collect data and to detach interrupts when the buffer is going to overflow
void interruptHandler1(void) {  
  //Do nothing
}

void setup() {
  // Begin serial connection and wait until it is activated
  Serial.begin(115200);
  while (!Serial);

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


  
void loop() {
    
}
