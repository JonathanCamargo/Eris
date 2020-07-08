#include "Arduino.h"
#include <SPI.h>
#include "ADS1256.h"

// Calls the STATUS register and then returns the first 4 bits, which contains the chip ID
uint8_t ADS1256::ADS1256_ReadChipID(void)
{
  uint8_t id;
  id = ADS1256_ReadReg(STATUS);
  return (id >> 4);
}

// Read a specific register address of the ADS1256 memory
uint8_t ADS1256::ADS1256_ReadReg(uint8_t _RegID)
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

// Write to a specific register address of the ADS1256 with a certain value
void ADS1256::ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(WREG << 4 | _RegID);
  SPI.transfer(0x00);
  SPI.transfer(_RegValue);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
}

// Configure the ADC gain and sampling rate registers
bool ADS1256::configADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);

  // Write using WREG command:
  // 1st byte: (WREG << 4)|rrrr  (rrrr is the addres of the first register to be writen)
  // 2nd byte: (0000 << 4)|nnnn (nnnn is the number of bytes to be writen-1)
  // ith byte: data
  
  // Configuration 1:
  //write
  SPIbuffer[0] = WREG << 4 | 0x00; 
  SPIbuffer[1] = 0x03; //Number of bytes-1
  SPIbuffer[2] = (0 << 3) | (1 << 2) | (1 << 1);  //STATUS: ORDER'  ACAL BUFEN
  SPIbuffer[3] = 0x08; // MUX: Psel=AIN0 NSEL=AINCOM
  SPIbuffer[4] = (0 << 5) | (0 << 3) | (_gain << 0); //ADCON: PGA2 PGA1 PGA2 (gain)
  SPIbuffer[5] = s_tabDataRate[_drate]; //A/D rate

  uint8_t regSettings[32];
  for (int i = 0; i < 6; i++) {
    regSettings[i] = SPIbuffer[i];
  }
  regSettings[2] = (3 << 4) | regSettings[2]; //add ID bits and shift (to check)
  
  transferBytes(6);

  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  // Delay to wait for calibration completion:
  
  delayMicroseconds(_drate*60000);
#define DEBUG true
  //check registers
  bool regCheck = true;
  for (int i = 0; i < 4; i++) {
    #if DEBUG
      Serial.print(regSettings[i + 2], BIN);
      Serial.print('\t');
      Serial.println(ADS1256::ADS1256_ReadReg(i), BIN);
    #endif
    
    regCheck = regCheck & (regSettings[i + 2] == ADS1256::ADS1256_ReadReg(i));
  }
  
  return regCheck;
}



