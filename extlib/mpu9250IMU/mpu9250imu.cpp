#include "Arduino.h"
#include <SPI.h>
#include "yostimu.h"

// Constructors
// Construct with only chip select pin
YostIMU::YostIMU(int cs):
   _cs(cs)
   #if USE_DMA
   ,
   csdma(ActiveLowChipSelect(cs, SPISETTINGS))   
   #endif
   {
  
  SPI.begin();         
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  #if USE_DMA
	  DMASPI0.begin();
	  DMASPI0.start();	  
  #endif

  //Reset the chip to defaults
  delay(10);
  reset();
  delay(10);
}


void YostIMU::ReadFloats(uint8_t *buffer,float * values, int numFloats){		
	//Take a list of binary data in a buffer and read floats from it
	uint8_t temp[4];
	for (uint8_t i=0;i<numFloats;i++){		
		temp[3]=buffer[0+i*4];
		temp[2]=buffer[1+i*4];
		temp[1]=buffer[2+i*4];
		temp[0]=buffer[3+i*4];
		memcpy(&values[i],temp,sizeof(float));
	}
}

void YostIMU::reset(){
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(RESET);    
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();  
}

void YostIMU::tare(){
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(TARE);    
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();  
}


YostIMU::~YostIMU(){
  SPI.endTransaction();
  SPI.end();
}




