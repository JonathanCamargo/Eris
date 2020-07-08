#include "Arduino.h"
#include <SPI.h>
#include "ADS1256.h"

//#define DEBUG_ADS1256
// Constructors
// Construct with only chip select pin, user handles drdy.
ADS1256::ADS1256(int cs){
  _cs = cs;
  bufferptrs[0] = &data_ch0;
  bufferptrs[1] = &data_ch1;
  bufferptrs[2] = &data_ch2;
  bufferptrs[3] = &data_ch3;
  bufferptrs[4] = &data_ch4;
  bufferptrs[5] = &data_ch5;
  bufferptrs[6] = &data_ch6;
  bufferptrs[7] = &data_ch7;
  SPI.begin();         
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  //Reset the chip to defaults
  delay(10);
  reset();
  delay(10);
 
  uint8_t id = ADS1256_ReadChipID();  
  
  if (id != 3)
  {
    Serial.print("Error, ADS1256 Chip ID = ");
    Serial.println(id);
    return;
  }
  else
  {
    #ifdef DEBUG_ADS1256
    Serial.println("Ok, ADS1256 Chip ID = 0x03");
    #endif
  }
}

// Constructor, must input CS pin and DRDY pin.
// configure ADC with gain=1/1 and sampling speed 2000SPS
ADS1256::ADS1256(int cs, int drdy, ADS1256_DRATE_E rate) : ADS1256(cs) {  
  _drdy = drdy; 
  pinMode(_drdy, INPUT_PULLUP);
     
  // Call configuration function
  if(configADC(ADS1256_GAIN_1, rate))
  {
    #ifdef DEBUG_ADS1256
    Serial.println("ADC configuration success");
    #endif
  }
  else
  {
    Serial.println("ADC configuration failed");
  }
}

// Construct with default sampling rate of 2kSPS
ADS1256::ADS1256(int cs, int drdy) : ADS1256(cs,drdy,ADS1256_2000SPS){
}

// update the number of channels to use
void ADS1256::setNumChannels(int n) {
	if (n<=8){
		this->numChannels = n;
	}
}

// Send multiple SPI bytes at once with correct waiting periods
void ADS1256::transferBytes(int numBytes)
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs, LOW);
  delayMicroseconds(2);
  for(int i = 0; i < numBytes; i++)
  {
    SPIbuffer[i] = SPI.transfer(SPIbuffer[i]);
    delayMicroseconds(2);
  }
  delayMicroseconds(2);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

//With multiple channels we can not run in continuous mode but this function is kept for 
// compatibility with single channel ino scripts
void ADS1256::startCollecting()
{  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(RDATAC);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();

  #ifdef DEBUG_ADS1256
  Serial.println("Starting ADC");
  #endif
}

//With multiple channels we can not run in continuous mode but this function is kept for 
// compatibility with single channel ino scripts
void ADS1256::stopCollecting()
{
  #ifdef DEBUG_ADS1256
  Serial.println("Stopping ADC");  
  #endif
}

// ISR that takes data from ADS1256 during continous read mode, converts the register values to the right value, and writes it to the current data buffer
// when the data buffer is full t7he function returns true to inform the user of ADS1256 about reading the data.
bool ADS1256::collectData(void){  
  // Multiple channels collection is explained in page 21 of datasheet.  
  uint8_t currentChannel=nextChannel;
  nextChannel=nextChannel+1;
  nextChannel= nextChannel > (numChannels - 1) ? 0 : nextChannel;
  
  // On data ready change the mux using WREG
  SPIbuffer[0] = WREG << 4 | MUX;
  SPIbuffer[1] = 0x00; // Number of bytes-1
  SPIbuffer[2] = (nextChannel << 4) | 0x08; // MUX: Psel=AIN0 NSEL=AINCOM
  transferBytes(3);
  // Then send a sync
  SPIbuffer[0] = SYNC;
  transferBytes(1);
  // Then send a wakeup
  SPIbuffer[0] = WAKEUP;
  transferBytes(1);
  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(RDATA);
  delayMicroseconds(20);
  SPIbuffer[0] = 0;
  SPIbuffer[1] = 0;
  SPIbuffer[2] = 0;
  SPI.transfer(SPIbuffer, 3);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  #if DEBUG_ADS1256
	  Serial.print(SPIbuffer[0],HEX);
	  Serial.print("-");
	  Serial.print(SPIbuffer[1],HEX);
	  Serial.print("-");
	  Serial.println(SPIbuffer[2],HEX);
  #endif
  
  uint32_t data = SPIbuffer[0] << 16 | SPIbuffer[1] << 8 | SPIbuffer[2];
  //Extending Two's compliment 24bit
  data=data^0x800000;  

  FastDualBuffer<float,ADS1256_BUFFERSIZE> * bufferptr=bufferptrs[currentChannel];
  float value = 2*VREF*(((float)data/8388607.0)-1);//Voltage
  bool isBufferFull=bufferptr->add(value);
  //bool isBufferFull=0;//=bufferptr->add(value);
  if (currentChannel == (numChannels - 1)){
    sampleCount=sampleCount+1;
    return isBufferFull;
  }
  return 0;
}


float* ADS1256::get_data(int channel) {
	if (channel<this->numChannels){
		return bufferptrs[channel]->read();
	}
	return NULL;
}

int ADS1256::get_sampleCount() {
  return sampleCount;
}

int ADS1256::bufferSize() {
  return ADS1256_BUFFERSIZE;
}

void ADS1256::reset()
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(RESET);    
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  //reset_data();  
}

//I don't think tthis function is really necessary but I'm keeping it just in case
void ADS1256::reset_data() 
{
   for (uint8_t i=0;i<this->numChannels;i++){
	FastDualBuffer<float,ADS1256_BUFFERSIZE> * bufferptr=bufferptrs[i];
	bufferptr->clear();
   }

}


// Get the first for registers which contains all the configuration registers
void ADS1256::get_setRegs() {
  uint8_t read[4];
  read[0] = ADS1256_ReadReg(0x00);
  Serial.println(read[0]);
  read[1] = ADS1256_ReadReg(0x01);
  Serial.println(read[1]);
  read[2] = ADS1256_ReadReg(0x02);
  Serial.println(read[2]);
  read[3] = ADS1256_ReadReg(0x03);
  Serial.println(read[3]);  
}

ADS1256::~ADS1256()
{
  SPI.endTransaction();
  SPI.end();
}


