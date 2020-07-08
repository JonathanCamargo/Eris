#include "Arduino.h"
#include <SPI.h>
#include "yostimu.h"

// Read a specific register address of the YostIMU memory
uint8_t YostIMU::Read(uint8_t *buffer,uint8_t dataSize)
{
  #if USE_DMA
	DMASPI0.begin();
	DMASPI0.start();
	digitalWrite(_cs,LOW);
	SPI.transfer(RREG);
	trx=DmaSpi::Transfer(nullptr, dataSize, buffer,0xFF,&csdma);
	DMASPI0.registerTransfer(trx);
	return true;

  #else
	SPI.beginTransaction(SPISETTINGS);
	digitalWrite(_cs,LOW);
	SPI.transfer(RREG);
	for (uint8_t i=0;i<dataSize;i++){
	buffer[i]=SPI.transfer(0xFF);
	}
	delayMicroseconds(12);
	digitalWrite(_cs,HIGH);
	SPI.endTransaction();
  	return true;
  #endif
}

bool YostIMU::DataReady(void){
	#if USE_DMA
	return trx.done();
	#else
	return true;
	#endif
}

// Write to a specific register address of the YostIMU with a certain value
void YostIMU::WriteCommand(uint8_t _cmd)
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(WREG);
  SPI.transfer(_cmd); 
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
}

// Write to a specific register address of the YostIMU with a series of values
void YostIMU::WriteCommand(uint8_t _cmd, uint8_t *_data,uint8_t dataSize)
{
  #if USE_DMA
	   this->WriteCommand(_cmd);
	   delayMicroseconds(1);
           trx=DmaSpi::Transfer(_data, dataSize, nullptr,0xFF,&csdma);
           DMASPI0.registerTransfer(trx);

  #else
	  SPI.beginTransaction(SPISETTINGS);
	  digitalWrite(_cs,LOW);
	  SPI.transfer(WREG);
	  SPI.transfer(_cmd);
	  for (uint8_t i=0;i<dataSize;i++){
		SPI.transfer(_data[i]);
	  }
	  digitalWrite(_cs,HIGH);
	  SPI.endTransaction();
  #endif
}



// Configure the ADC gain and sampling rate registers




