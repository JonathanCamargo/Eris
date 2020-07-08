#include "Arduino.h"
#include <SPI.h>
#include "ADS1299.h"

// Calls the STATUS register and then returns the first 4 bits, which contains the chip ID
uint8_t ADS1299::ADS1299_ReadChipID(void)
{
  uint8_t id;
  id = ADS1299_ReadReg(STATUS);
  return (id >> 4);
}

// Read a specific register address of the ADS1299 memory
uint8_t ADS1299::ADS1299_ReadReg(uint8_t _RegID)
{
  uint8_t read;

  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(RREG << 4 | _RegID);
  SPI.transfer(0x00);
  delayMicroseconds(12);
  read = SPI.transfer(0xFF);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();

  return read;
}

// Write to a specific register address of the ADS1299 with a certain value
void ADS1299::ADS1299_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(WREG << 4 | _RegID);
  SPI.transfer(0x00);
  SPI.transfer(_RegValue);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
}

// Compare a content buffer vs the registers read from the ADS
bool ADS1299::regCompare(uint8_t * regSettings,uint8_t regAddr,uint8_t len) {  
  //check registers
  bool regCheck = true;  
  for (int i = 0; i < len; i++) {
    #ifdef DEBUG
      Serial.print("(REG");
      Serial.print(regAddr+i,HEX);
      Serial.print(")");
      Serial.print(regSettings[i], BIN);
      uint8_t r=ADS1299::ADS1299_ReadReg(regAddr+i);
      Serial.print('\t');
      Serial.println(r, BIN);
    #endif   
    regCheck = regCheck & (regSettings[i] == r);
  }
  return regCheck;
}

// Configure the ADC gain and sampling rate registers
bool ADS1299::configADC(ADS1299_GAIN_E _gain, ADS1299_DRATE_E _drate)
{
  this->gain=s_tabDataGain[_gain];
  Serial.println("gain");
  Serial.println(_gain,HEX);
  
  // Write using WREG command:
  // 1st byte: (WREG << 4)|rrrr  (rrrr is the addres of the first register to be writen)
  // 2nd byte: (0000 << 4)|nnnn (nnnn is the number of bytes to be writen-1)
  // ith byte: data  
  // Configuration 1:
  //write
  uint8_t regSettings[64]; //Buffer to store the settings  
  bool regCheck=true;
  //******************************************************//
  // First set up the config registers and leadoff
  SPIbuffer[0] = WREG << 4 | CONFIG1_ID; 
  SPIbuffer[1] = 4-1; //Number of bytes-1
  SPIbuffer[2] = (0x01<<7)| (0x02 << 3) | (s_tabDataRate[_drate]);  // CONFIG 1
  SPIbuffer[3] = (0x06<<5) | (0x01<<4) | (0x01 <<2) | (0x01); // CONFIG2
  SPIbuffer[4] =  (0x01<<7) | (0x03 << 5) | (0x01 << 3) | (0x01<<2); // CONFIG3
  SPIbuffer[5] = 0x00; // Leadoff
  
  for (int i = 0; i < 4; i++) {
    regSettings[i] = SPIbuffer[i+2];
  }
  
  SPI.beginTransaction(SPISETTINGS);  
  digitalWrite(_cs,LOW);
  transferBytes(4+2);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();

  regCheck=regCheck & regCompare(regSettings,CONFIG1_ID,4);  
  //******************************************************//


  

  ////////////////////////////////////////////////////////////
  // Then configure channels
  SPIbuffer[0] = WREG << 4 | CH0_ID; 
  SPIbuffer[1] = 8-1; //Number of bytes-1
  SPIbuffer[2] = (_gain <<4) | (0x01);  // Ch0   // (0x05) TO TEST // (0x01) to short
  SPIbuffer[3] = (_gain <<4) | (0x00);  // Ch1   // TO COLLECT
  SPIbuffer[4] = (_gain <<4) | (0x01);  // Ch2   // TO COLLECT
  SPIbuffer[5] = (_gain <<4) | (0x01);  // Ch3   // TO COLLECT
  //SPIbuffer[6] = (_gain <<4) | (0x00);  // Ch4   // TO COLLECT
  //SPIbuffer[7] = (_gain <<4) | (0x00);  // Ch5   // TO COLLECT
  //SPIbuffer[8] = (_gain <<4) | (0x00);  // Ch6   // TO COLLECT
  //SPIbuffer[9] = (_gain <<4) | (0x00);  // Ch7   // TO COLLECT  
  for (int i = 0; i < 4; i++) {
    regSettings[i] = SPIbuffer[i+2];
  }  
  SPI.beginTransaction(SPISETTINGS);  
  digitalWrite(_cs,LOW);
  transferBytes(4+2); 
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();  
  regCheck=regCheck & regCompare(regSettings,CH0_ID,4);  
  ////////////////////////////////////////////////////////////////


 /////////////////////////////////////////////////////////////////
  // Then configure the Bias 
  SPIbuffer[0] = WREG << 4 | BIAS_SENSP; 
  SPIbuffer[1] = 2-1; //Number of bytes-1
  SPIbuffer[2] = 0x02;  // BIAS_SENSP
  SPIbuffer[3] = 0x02;  // BIAS_SENSN 
  
  for (int i = 0; i < 2; i++) {
    regSettings[i] = SPIbuffer[i+2];
  }
  
  SPI.beginTransaction(SPISETTINGS);  
  digitalWrite(_cs,LOW);
  transferBytes(2+2);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  
  regCheck=regCheck & regCompare(regSettings,BIAS_SENSP,2);  

  /////////////////////////////////////////////////////////////////
  

  
  // Delay to wait for calibration completion:  
  delayMicroseconds(_drate*60000);

  
  
  return regCheck;
}
