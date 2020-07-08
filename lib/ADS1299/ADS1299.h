// ADS1299 Teensy Library for use with NextFlex sensor
/*
To use this library, put include path and include "ADS1299.h". The
class "ADS1299" contains all functions necessary to opperate the ADS1299 with
Teensy. To use the object, Initialize an ADS1299 object with the Teensy CS pin
and DRDY pin connected to the ADS1299. Then, create a static void function in
the Teensy code that simply calls the "collectData" function from the object. It
should look something like this:

static void interruptHandler(void) {
  sensor1.collectData();
}

To start collecting data, call the "startCollecting" function for the object to
put the ADS1299 chip in collecting mode and attachInterrupt the static void
function that you made to the DRDY pin. This will then start collecting data in
a file named "EMG#.csv" where # will always be the lowest number that has not
already been taken on the SD card. To stop collecting data, detach the interrupt
and call "stopCollecting" to turn the ADS1299 back to normal mode.
*/

#ifndef ADS1299_teensy_lib_v2_h
#define ADS1299_teensy_lib_v2_h

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include <fastdualbuffer.h>

#define VREF 1.2

#define DEBUG
#define DEBUG_ADS1299

#define ADS1299_BUFFERSIZE 10

#define SPISETTINGS SPISettings(2000000,MSBFIRST, SPI_MODE1)

//Registers
#define STATUS 0x00
#define MUX	0x01
#define CONFIG1_ID 0x01
#define CONFIG2_ID 0x02
#define CONFIG3_ID 0x03
#define FAULT_ID 0x04
#define CH0_ID 0x05
#define CH1_ID 0x06
#define CH2_ID 0x07
#define CH3_ID 0x08
#define CH4_ID 0x09
#define CH5_ID 0x0A
#define CH6_ID 0x0B
#define CH7_ID 0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E

#define FAULT_STATP 0x12
#define FAULT_STATN 0x13
#define GPIO 0x14

//commands
#define WAKEUP 0x02
#define RDATA 0x12
#define RDATAC 0x10
#define SDATAC 0x11

#define RREG 0x02
#define WREG 0x04

#define STANDBY 0x04
#define RESET 0x06
#define START 0x08
#define STOP 0x0A
#define OFFSETCAL 0xF3
#define GAINCAL 0XF2
#define SYSOFFSETCAL 0xF3
#define SYSGAINCAL 0xF4
#define SYNC 0xFC




//Gain Enum
typedef enum
{
  ADS1299_GAIN_1  = 0  ,  /* GAIN   1 */
  ADS1299_GAIN_2      ,  /*GAIN   2 */
  ADS1299_GAIN_4       ,  /*GAIN   4 */
  ADS1299_GAIN_6      ,  /*GAIN   6 */
  ADS1299_GAIN_8      ,  /*GAIN   8 */
  ADS1299_GAIN_12      ,  /* GAIN  12 */
  ADS1299_GAIN_24  = 6   ,  /*GAIN    24 */  
  ADS1299_DGAIN_MAX
}ADS1299_GAIN_E;


static const uint8_t s_tabDataGain[ADS1299_DGAIN_MAX] =
{
  1,   
  2,
  4,
  6,
  8,
  12,
  24  
};


//Sampling Speed Enum
typedef enum
{
  ADS1299_16000SPS = 0,
  ADS1299_8000SPS,
  ADS1299_4000SPS,
  ADS1299_2000SPS,
  ADS1299_1000SPS,/* Default*/ 
  ADS1299_500SPS, 
  ADS1299_250SPS,  
  ADS1299_DRATE_MAX
}ADS1299_DRATE_E;

#define ADS1299_DRATE_COUNT = 15;

typedef struct
{
  ADS1299_GAIN_E Gain;    /* GAIN  */
  ADS1299_DRATE_E DataRate; /* DATA output  speed*/
  int32_t AdcNow[8];      /* ADC  Conversion value */
  uint8_t Channel;      /* The current channel*/
  uint8_t ScanMode; /*Scanning mode,   0  Single-ended input  8 channelÃ¯Â¿Â½Ã¯Â¿Â½ 1 Differential input  4 channel*/
}ADS1299_VAR_T;

static const uint8_t s_tabDataRate[ADS1299_DRATE_MAX] =
{
  0x00,   
  0x01,
  0x02,
  0x03,
  0x04,
  0x05,
  0x06   /* Default*/  
};



class ADS1299 {
  public:

  ADS1299(int cs);
  ADS1299(int cs, int drdy); 
  ADS1299(int cs, int drdy, ADS1299_DRATE_E _drate);

  void setNumChannels(int n); 
  void startCollecting();
  void stopCollecting(void);
  bool collectData(void);
  bool regCompare(uint8_t*,uint8_t,uint8_t);
  
  bool configADC(ADS1299_GAIN_E _gain, ADS1299_DRATE_E _drate);

  float* get_data(int channel);
  
  uint8_t ADS1299_ReadReg(uint8_t _RegID);
  void ADS1299_WriteReg(uint8_t _RegID, uint8_t _RegValue);
  
  int bufferSize(void);

  void reset(void);
  
  ~ADS1299();

  private: 
  int numChannels = 4;
  uint8_t gain=0;
  
	
  float data0_ch0[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch0[ADS1299_BUFFERSIZE]; //Second buffer
  float data0_ch1[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch1[ADS1299_BUFFERSIZE]; //Second buffer
  float data0_ch2[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch2[ADS1299_BUFFERSIZE]; //Second buffer
  float data0_ch3[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch3[ADS1299_BUFFERSIZE]; //Second buffer

  //Change to use fastdualbuffer?



  float* _data_ch0; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch0; //Data stored (buffer where previous data is stored
  float* _data_ch1; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch1; //Data stored (buffer where previous data is stored
  float* _data_ch2; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch2; //Data stored (buffer where previous data is stored
  float* _data_ch3; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch3; //Data stored (buffer where previous data is stored

  bool bufferIdx; // which buffer is active (i.e. catching data) 0 or 1
  
  uint8_t ADS1299_ReadChipID(void);
  void transferBytes(int numBytes);
  int _cs;
  int _drdy;
  uint8_t SPIbuffer[32];
  uint16_t sampleCount = 0;
  uint8_t nextChannel =0;
  
  void reset_data(void);  
  int get_sampleCount(void);
  void get_setRegs(void);
  

};




#endif
=======
// ADS1299 Teensy Library for use with NextFlex sensor
/*
To use this library, put include path and include "ADS1299.h". The
class "ADS1299" contains all functions necessary to opperate the ADS1299 with
Teensy. To use the object, Initialize an ADS1299 object with the Teensy CS pin
and DRDY pin connected to the ADS1299. Then, create a static void function in
the Teensy code that simply calls the "collectData" function from the object. It
should look something like this:

static void interruptHandler(void) {
  sensor1.collectData();
}

To start collecting data, call the "startCollecting" function for the object to
put the ADS1299 chip in collecting mode and attachInterrupt the static void
function that you made to the DRDY pin. This will then start collecting data in
a file named "EMG#.csv" where # will always be the lowest number that has not
already been taken on the SD card. To stop collecting data, detach the interrupt
and call "stopCollecting" to turn the ADS1299 back to normal mode.
*/

#ifndef ADS1299_teensy_lib_v2_h
#define ADS1299_teensy_lib_v2_h

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include <fastdualbuffer.h>

#define VREF 1.2

#define DEBUG
#define DEBUG_ADS1299

#define ADS1299_BUFFERSIZE 1

#define SPISETTINGS SPISettings(10000000,MSBFIRST, SPI_MODE1)

//Registers
#define STATUS 0x00
#define MUX	0x01
#define CONFIG1_ID 0x01
#define CONFIG2_ID 0x02
#define CONFIG3_ID 0x03
#define FAULT_ID 0x04
#define CH0_ID 0x05
#define CH1_ID 0x06
#define CH2_ID 0x07
#define CH3_ID 0x08
#define CH4_ID 0x09
#define CH5_ID 0x0A
#define CH6_ID 0x0B
#define CH7_ID 0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E

#define FAULT_STATP 0x12
#define FAULT_STATN 0x13
#define GPIO 0x14

//commands
#define WAKEUP 0x02
#define RDATA 0x12
#define RDATAC 0x10
#define SDATAC 0x11

#define RREG 0x02
#define WREG 0x04

#define STANDBY 0x04
#define RESET 0x06
#define START 0x08
#define STOP 0x0A
#define OFFSETCAL 0xF3
#define GAINCAL 0XF2
#define SYSOFFSETCAL 0xF3
#define SYSGAINCAL 0xF4
#define SYNC 0xFC




//Gain Enum
typedef enum
{
  ADS1299_GAIN_1  = 0  ,  /* GAIN   1 */
  ADS1299_GAIN_2      ,  /*GAIN   2 */
  ADS1299_GAIN_4       ,  /*GAIN   4 */
  ADS1299_GAIN_6      ,  /*GAIN   6 */
  ADS1299_GAIN_8      ,  /*GAIN   8 */
  ADS1299_GAIN_12      ,  /* GAIN  12 */
  ADS1299_GAIN_24  = 6   ,  /*GAIN    24 */  
  ADS1299_DGAIN_MAX
}ADS1299_GAIN_E;


static const uint8_t s_tabDataGain[ADS1299_DGAIN_MAX] =
{
  1,   
  2,
  4,
  6,
  8,
  12,
  24  
};


//Sampling Speed Enum
typedef enum
{
  ADS1299_16000SPS = 0,
  ADS1299_8000SPS,
  ADS1299_4000SPS,
  ADS1299_2000SPS,
  ADS1299_1000SPS,/* Default*/ 
  ADS1299_500SPS, 
  ADS1299_250SPS,  
  ADS1299_DRATE_MAX
}ADS1299_DRATE_E;

#define ADS1299_DRATE_COUNT = 15;

typedef struct
{
  ADS1299_GAIN_E Gain;    /* GAIN  */
  ADS1299_DRATE_E DataRate; /* DATA output  speed*/
  int32_t AdcNow[8];      /* ADC  Conversion value */
  uint8_t Channel;      /* The current channel*/
  uint8_t ScanMode; /*Scanning mode,   0  Single-ended input  8 channelÃ¯Â¿Â½Ã¯Â¿Â½ 1 Differential input  4 channel*/
}ADS1299_VAR_T;

static const uint8_t s_tabDataRate[ADS1299_DRATE_MAX] =
{
  0x00,   
  0x01,
  0x02,
  0x03,
  0x04,
  0x05,
  0x06   /* Default*/  
};



class ADS1299 {
  public:

  ADS1299(int cs);
  ADS1299(int cs, int drdy); 
  ADS1299(int cs, int drdy, ADS1299_DRATE_E _drate);

  void setNumChannels(int n); 
  void startCollecting();
  void stopCollecting(void);
  bool collectData(void);
  
  
  bool regCompare(uint8_t*,uint8_t,uint8_t);
  
  bool configADC(ADS1299_GAIN_E _gain, ADS1299_DRATE_E _drate);

  float* get_data(int channel);
  
  uint8_t ADS1299_ReadReg(uint8_t _RegID);
  void ADS1299_WriteReg(uint8_t _RegID, uint8_t _RegValue);
  
  int bufferSize(void);

  void reset(void);
  
  ~ADS1299();

  private: 
  int numChannels = 4;
  uint8_t gain=0;
  
  //bool ReadData(void);
	
  float data0_ch0[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch0[ADS1299_BUFFERSIZE]; //Second buffer
  float data0_ch1[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch1[ADS1299_BUFFERSIZE]; //Second buffer
  float data0_ch2[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch2[ADS1299_BUFFERSIZE]; //Second buffer
  float data0_ch3[ADS1299_BUFFERSIZE]; //First buffer
  float data1_ch3[ADS1299_BUFFERSIZE]; //Second buffer

  //Change to use fastdualbuffer?



  float* _data_ch0; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch0; //Data stored (buffer where previous data is stored
  float* _data_ch1; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch1; //Data stored (buffer where previous data is stored
  float* _data_ch2; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch2; //Data stored (buffer where previous data is stored
  float* _data_ch3; //Active buffer (buffer where it is currently saving data to)
  float* _storedData_ch3; //Data stored (buffer where previous data is stored

  bool bufferIdx; // which buffer is active (i.e. catching data) 0 or 1
  
  uint8_t ADS1299_ReadChipID(void);
  void transferBytes(int numBytes);
  int _cs;
  int _drdy;
  uint8_t SPIbuffer[32];
  uint16_t sampleCount = 0;
  uint8_t nextChannel =0;
  
  void reset_data(void);  
  int get_sampleCount(void);
  void get_setRegs(void);
  

};




#endif