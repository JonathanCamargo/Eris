#include "Arduino.h"
#include <SPI.h>
#include "ADS1299.h"

//#define DEBUG_ADS1299
// Constructors
// Construct with only chip select pin, user handles drdy.
ADS1299::ADS1299(int cs){
  _cs = cs;
  _data_ch0 = data0_ch0;
  _data_ch1 = data0_ch1;
  _data_ch2 = data0_ch2;
  _data_ch3 = data0_ch3;

  _storedData_ch0=NULL;
  _storedData_ch1=NULL;
  _storedData_ch2=NULL;
  _storedData_ch3=NULL;

  #ifdef DEBUG_ADS1299
  if (!Serial) {
    Serial.begin(115200);
    while (!Serial);
  }
  #endif
  SPI.begin();         
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  //Reset the chip to defaults
  delay(10);
  reset();
  delay(10);
 
  uint8_t id = ADS1299_ReadChipID();  
  if (id != 3)
  {
    Serial.print("Error, ADS1299 Chip ID = ");
    Serial.println(id);
    return;
  }
  else
  {
    #ifdef DEBUG_ADS1299
    Serial.println("Ok, ADS1299 Chip ID = 0x03");
    #endif
  }  
}

// Constructor, must input CS pin and DRDY pin.
// configure ADC with gain=1/1 and sampling speed 1000SPS
ADS1299::ADS1299(int cs, int drdy, ADS1299_DRATE_E rate) : ADS1299(cs) {  
  _drdy = drdy; 
  pinMode(_drdy, INPUT);
 
  setNumChannels(4);
     
  // Call configuration function
  if(configADC(ADS1299_GAIN_24, rate))
  {
    #ifdef DEBUG_ADS1299
    Serial.println("ADC configuration success");
    #endif
  }
  else
  {
    Serial.println("ADC configuration failed");
  }  
}

// Construct with default sampling rate of 2kSPS
ADS1299::ADS1299(int cs, int drdy) : ADS1299(cs,drdy,ADS1299_1000SPS){
}

// update the number of channels to use
void ADS1299::setNumChannels(int n) {
	numChannels = n;
}

// Send multiple SPI bytes at once with correct waiting periods
void ADS1299::transferBytes(int numBytes)
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
void ADS1299::startCollecting()
{  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(START);   
  delayMicroseconds(3);  
  delayMicroseconds(3);    
  SPI.transfer(RDATAC);   
  delayMicroseconds(3);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();

  #ifdef DEBUG_ADS1299
  Serial.println("Starting ADC");
  #endif
}

//With multiple channels we can not run in continuous mode but this function is kept for 
// compatibility with single channel ino scripts
void ADS1299::stopCollecting()
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(SDATAC);    
  delayMicroseconds(3);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  #ifdef DEBUG_ADS1299
  Serial.println("Stopping ADC");  
  #endif
}

// ISR that takes data from ADS1299 during continous read mode, converts the register values to the right value, and writes it to the current data buffer
// when the data buffer is full the function returns true to inform the user of ADS1299 about reading the data.
bool ADS1299::collectData(void){  
    
  for (uint8_t i=0;i<27;i++){
    SPIbuffer[i] = 0;
  }  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW); 
  SPI.transfer(SPIbuffer, 27); 
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();  

 
  float * _data;
  uint8_t channel=0;
  uint8_t startAt=channel*3+3;
  uint32_t data;
  if (numChannels>0){
	 // Read first channel  
	 //Serial.println(SPIbuffer[3],BIN);
         data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
	 //Extending Two's compliment 24bit
 	 data=data^0x800000;  
	 /*Serial.print(SPIbuffer[0],HEX);
	  Serial.print(" ");
	  Serial.print(SPIbuffer[1],HEX);
	  Serial.print(" ");
	  Serial.print(SPIbuffer[2],HEX);
	  Serial.print("--");
	  Serial.print(SPIbuffer[3],HEX);
	  Serial.print(" ");
	  Serial.print(SPIbuffer[4],HEX);
	  Serial.print(" ");
	  Serial.println(SPIbuffer[5],HEX);
	  //Serial.print(" ");
	  //Serial.println(SPIbuffer[6],HEX);
	 */
	 _data=_data_ch0;		
	 _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV 
  }
  if (numChannels>1){
       channel=1;
       startAt=channel*3+3;
       data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
       //Extending Two's compliment 24bit
       data=data^0x800000;  
       _data=_data_ch1;		
       _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV
  }
  if (numChannels>2){
       channel=2;
       startAt=channel*3+3;
       data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
       //Extending Two's compliment 24bit
       data=data^0x800000;  
       _data=_data_ch2;		
       _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV
  }
  if (numChannels>3){
       channel=3;
       startAt=channel*3+3;
       data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
       //Extending Two's compliment 24bit
       data=data^0x800000;  
       _data=_data_ch3;		
       _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV
  }

  //*Serial.println(_data[sampleCount]);
  sampleCount=sampleCount+1;
    
  if (sampleCount>=ADS1299_BUFFERSIZE){	
    this->bufferIdx= !(this->bufferIdx);
    _data_ch0= bufferIdx == false ? data0_ch0 : data1_ch0 ;
    _storedData_ch0= bufferIdx == true ? data0_ch0 : data1_ch0 ;
    _data_ch1= bufferIdx == false ? data0_ch1 : data1_ch1 ;
    _storedData_ch1= bufferIdx == true ? data0_ch1 : data1_ch1 ;
    _data_ch2= bufferIdx == false ? data0_ch2 : data1_ch2 ;
    _storedData_ch2= bufferIdx == true ? data0_ch2 : data1_ch2 ;
    _data_ch3= bufferIdx == false ? data0_ch3 : data1_ch3 ;
    _storedData_ch3= bufferIdx == true ? data0_ch3 : data1_ch3 ;
    sampleCount=0;    
    return true;      
  }
  else{
    return false;
  } 
}


float* ADS1299::get_data(int channel) {
  switch (channel){
	case 0:
	  return _storedData_ch0;
	case 1:
	  return _storedData_ch1;
	case 2:
	  return _storedData_ch2;
	case 3:
	  return _storedData_ch3;
	default:
	  return _storedData_ch0;
  }
}

int ADS1299::get_sampleCount() {
  return sampleCount;
}

int ADS1299::bufferSize() {
  return ADS1299_BUFFERSIZE;
}

void ADS1299::reset()
{

  SPI.beginTransaction(SPISETTINGS);
  
  digitalWrite(_cs,LOW);
  SPI.transfer(RESET);    
  delayMicroseconds(12);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(SDATAC);    
  delayMicroseconds(3);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  
  reset_data();
  _data_ch0 = data0_ch0;
  _data_ch1 = data0_ch1;
  _data_ch2 = data0_ch2;
  _data_ch3 = data0_ch3;

  _storedData_ch0=NULL;
  _storedData_ch1=NULL;
  _storedData_ch2=NULL;
  _storedData_ch3=NULL;
}

//I don't think tthis function is really necessary but I'm keeping it just in case
void ADS1299::reset_data() 
{
  memset(data0_ch0,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch0,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data0_ch1,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch1,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data0_ch2,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch2,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data0_ch3,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch3,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));

}


// Get the first for registers which contains all the configuration registers
void ADS1299::get_setRegs() {
  uint8_t read[4];
  read[0] = ADS1299_ReadReg(0x00);
  Serial.println(read[0]);
  read[1] = ADS1299_ReadReg(0x01);
  Serial.println(read[1]);
  read[2] = ADS1299_ReadReg(0x02);
  Serial.println(read[2]);
  read[3] = ADS1299_ReadReg(0x03);
  Serial.println(read[3]);  
}

ADS1299::~ADS1299()
{
  SPI.endTransaction();
  SPI.end();
}
=======
#include "Arduino.h"
#include <SPI.h>
#include "ADS1299.h"

//#define DEBUG_ADS1299
// Constructors
// Construct with only chip select pin, user handles drdy.
ADS1299::ADS1299(int cs){
  _cs = cs;
  _data_ch0 = data0_ch0;
  _data_ch1 = data0_ch1;
  _data_ch2 = data0_ch2;
  _data_ch3 = data0_ch3;

  _storedData_ch0=NULL;
  _storedData_ch1=NULL;
  _storedData_ch2=NULL;
  _storedData_ch3=NULL;

  #ifdef DEBUG_ADS1299
  if (!Serial) {
    Serial.begin(115200);
    while (!Serial);
  }
  #endif
  SPI.begin();         
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  //Reset the chip to defaults
  delay(10);
  reset();
  delay(10);
 
  uint8_t id = ADS1299_ReadChipID();  
  if (id != 3)
  {
    Serial.print("Error, ADS1299 Chip ID = ");
    Serial.println(id);
    return;
  }
  else
  {
    #ifdef DEBUG_ADS1299
    Serial.println("Ok, ADS1299 Chip ID = 0x03");
    #endif
  }  
}

// Constructor, must input CS pin and DRDY pin.
// configure ADC with gain=1/1 and sampling speed 1000SPS
ADS1299::ADS1299(int cs, int drdy, ADS1299_DRATE_E rate) : ADS1299(cs) {  
  _drdy = drdy; 
  pinMode(_drdy, INPUT);
 
  setNumChannels(4);
     
  // Call configuration function
  if(configADC(ADS1299_GAIN_24, rate))
  {
    #ifdef DEBUG_ADS1299
    Serial.println("ADC configuration success");
    #endif
  }
  else
  {
    Serial.println("ADC configuration failed");
  }  
}

// Construct with default sampling rate of 2kSPS
ADS1299::ADS1299(int cs, int drdy) : ADS1299(cs,drdy,ADS1299_1000SPS){
}

// update the number of channels to use
void ADS1299::setNumChannels(int n) {
	numChannels = n;
}

// Send multiple SPI bytes at once with correct waiting periods
void ADS1299::transferBytes(int numBytes)
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
void ADS1299::startCollecting()
{  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(START);   
  delayMicroseconds(3);  
  delayMicroseconds(3);    
  SPI.transfer(RDATAC);   
  delayMicroseconds(3);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();

  #ifdef DEBUG_ADS1299
  Serial.println("Starting ADC");
  #endif
}

//With multiple channels we can not run in continuous mode but this function is kept for 
// compatibility with single channel ino scripts
void ADS1299::stopCollecting()
{
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(SDATAC);    
  delayMicroseconds(3);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  #ifdef DEBUG_ADS1299
  Serial.println("Stopping ADC");  
  #endif
}


// ISR that takes data from ADS1299 during continous read mode, converts the register values to the right value, and writes it to the current data buffer
// when the data buffer is full the function returns true to inform the user of ADS1299 about reading the data.
bool ADS1299::collectData(void){  
    
  for (uint8_t i=0;i<27;i++){
    SPIbuffer[i] = 0;
  }  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW); 
  SPI.transfer(SPIbuffer, 27); 
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();  

 
  float * _data;
  uint8_t channel=0;
  uint8_t startAt=channel*3+3;
  uint32_t data;
  if (numChannels>0){
	 // Read first channel  
	 //Serial.println(SPIbuffer[3],BIN);
         data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
	 //Extending Two's compliment 24bit
 	 data=data^0x800000;  
	 /*Serial.print(SPIbuffer[0],HEX);
	  Serial.print(" ");
	  Serial.print(SPIbuffer[1],HEX);
	  Serial.print(" ");
	  Serial.print(SPIbuffer[2],HEX);
	  Serial.print("--");
	  Serial.print(SPIbuffer[3],HEX);
	  Serial.print(" ");
	  Serial.print(SPIbuffer[4],HEX);
	  Serial.print(" ");
	  Serial.println(SPIbuffer[5],HEX);
	  //Serial.print(" ");
	  //Serial.println(SPIbuffer[6],HEX);
	 */
	 _data=_data_ch0;		
	 _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV 
  }
  if (numChannels>1){
       channel=1;
       startAt=channel*3+3;
       data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
       //Extending Two's compliment 24bit
       data=data^0x800000;  
       _data=_data_ch1;		
       _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV
  }
  if (numChannels>2){
       channel=2;
       startAt=channel*3+3;
       data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
       //Extending Two's compliment 24bit
       data=data^0x800000;  
       _data=_data_ch2;		
       _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV
  }
  if (numChannels>3){
       channel=3;
       startAt=channel*3+3;
       data = SPIbuffer[startAt+0] << 16 | SPIbuffer[startAt+1] << 8 | SPIbuffer[startAt+2];
       //Extending Two's compliment 24bit
       data=data^0x800000;  
       _data=_data_ch3;		
       _data[sampleCount] = 1000*(2.0*(float)VREF/this->gain)*(((float)data/8388607.0)-1);//Voltage mV
  }

  //*Serial.println(_data[sampleCount]);
  sampleCount=sampleCount+1;
    
  if (sampleCount>=ADS1299_BUFFERSIZE){	
    this->bufferIdx= !(this->bufferIdx);
    _data_ch0= bufferIdx == false ? data0_ch0 : data1_ch0 ;
    _storedData_ch0= bufferIdx == true ? data0_ch0 : data1_ch0 ;
    _data_ch1= bufferIdx == false ? data0_ch1 : data1_ch1 ;
    _storedData_ch1= bufferIdx == true ? data0_ch1 : data1_ch1 ;
    _data_ch2= bufferIdx == false ? data0_ch2 : data1_ch2 ;
    _storedData_ch2= bufferIdx == true ? data0_ch2 : data1_ch2 ;
    _data_ch3= bufferIdx == false ? data0_ch3 : data1_ch3 ;
    _storedData_ch3= bufferIdx == true ? data0_ch3 : data1_ch3 ;
    sampleCount=0;    
    return true;      
  }
  else{
    return false;
  } 
}


float* ADS1299::get_data(int channel) {
  switch (channel){
	case 0:
	  return _storedData_ch0;
	case 1:
	  return _storedData_ch1;
	case 2:
	  return _storedData_ch2;
	case 3:
	  return _storedData_ch3;
	default:
	  return _storedData_ch0;
  }
}

int ADS1299::get_sampleCount() {
  return sampleCount;
}

int ADS1299::bufferSize() {
  return ADS1299_BUFFERSIZE;
}

void ADS1299::reset()
{

  SPI.beginTransaction(SPISETTINGS);
  
  digitalWrite(_cs,LOW);
  SPI.transfer(RESET);    
  delayMicroseconds(12);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  
  SPI.beginTransaction(SPISETTINGS);
  digitalWrite(_cs,LOW);
  SPI.transfer(SDATAC);    
  delayMicroseconds(3);
  digitalWrite(_cs,HIGH);
  SPI.endTransaction();
  
  reset_data();
  _data_ch0 = data0_ch0;
  _data_ch1 = data0_ch1;
  _data_ch2 = data0_ch2;
  _data_ch3 = data0_ch3;

  _storedData_ch0=NULL;
  _storedData_ch1=NULL;
  _storedData_ch2=NULL;
  _storedData_ch3=NULL;
}

//I don't think tthis function is really necessary but I'm keeping it just in case
void ADS1299::reset_data() 
{
  memset(data0_ch0,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch0,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data0_ch1,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch1,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data0_ch2,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch2,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data0_ch3,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));
  memset(data1_ch3,0,sizeof(ADS1299_BUFFERSIZE*sizeof(float)));

}


// Get the first for registers which contains all the configuration registers
void ADS1299::get_setRegs() {
  uint8_t read[4];
  read[0] = ADS1299_ReadReg(0x00);
  Serial.println(read[0]);
  read[1] = ADS1299_ReadReg(0x01);
  Serial.println(read[1]);
  read[2] = ADS1299_ReadReg(0x02);
  Serial.println(read[2]);
  read[3] = ADS1299_ReadReg(0x03);
  Serial.println(read[3]);  
}

ADS1299::~ADS1299()
{
  SPI.endTransaction();
  SPI.end();
}