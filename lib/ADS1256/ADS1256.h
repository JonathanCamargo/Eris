// ADS1256 Teensy Library for use with NextFlex sensor
/*
To use this library, put include path and include "ADS1256.h". The
class "ADS1256" contains all functions necessary to opperate the ADS1256 with
Teensy. To use the object, Initialize an ADS1256 object with the Teensy CS pin
and DRDY pin connected to the ADS1256. Then, create a static void function in
the Teensy code that simply calls the "collectData" function from the object. It
should look something like this:

static void interruptHandler(void) {
  sensor1.collectData();
}

To start collecting data, call the "startCollecting" function for the object to
put the ADS1256 chip in collecting mode and attachInterrupt to the static void
function that you made to the DRDY pin. To stop collecting data, detach the interrupt
and call "stopCollecting" to turn the ADS1256 back to normal mode.
*/

#ifndef ADS1256_teensy_lib_v2_h
#define ADS1256_teensy_lib_v2_h

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include <fastdualbuffer.h>

#define VREF 2.50

#define DEBUG_ADS1256 false

#define ADS1256_BUFFERSIZE 10

#define SPISETTINGS SPISettings(1000000, MSBFIRST, SPI_MODE1)

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
#define FAULT_STATP 0x12
#define FAULT_STATN 0x13
#define GPIO 0x14

//commands
#define WAKEUP 0x00
#define RDATA 0x01
#define RDATAC 0x03
#define SDATAC 0x0F

#define RREG 0x01
#define WREG 0x05

#define STANDBY 0xFD
#define RESET 0xFE
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
  ADS1256_GAIN_1      = (0),  /* GAIN   1 */
  ADS1256_GAIN_2      = (1),  /*GAIN   2 */
  ADS1256_GAIN_4      = (2),  /*GAIN   4 */
  ADS1256_GAIN_8      = (3),  /*GAIN   8 */
  ADS1256_GAIN_16     = (4),  /* GAIN  16 */
  ADS1256_GAIN_32     = (5),  /*GAIN    32 */
  ADS1256_GAIN_64     = (6),  /*GAIN    64 */
}ADS1256_GAIN_E;

//Sampling Speed Enum
typedef enum
{
  ADS1256_30000SPS = 0,
  ADS1256_15000SPS,
  ADS1256_7500SPS,
  ADS1256_3750SPS,
  ADS1256_2000SPS,
  ADS1256_1000SPS,
  ADS1256_500SPS,
  ADS1256_100SPS,
  ADS1256_60SPS,
  ADS1256_50SPS,
  ADS1256_30SPS,
  ADS1256_25SPS,
  ADS1256_15SPS,
  ADS1256_10SPS,
  ADS1256_5SPS,
  ADS1256_2d5SPS,

  ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRATE_COUNT = 15;

typedef struct
{
  ADS1256_GAIN_E Gain;    /* GAIN  */
  ADS1256_DRATE_E DataRate; /* DATA output  speed*/
  int32_t AdcNow[8];      /* ADC  Conversion value */
  uint8_t Channel;      /* The current channel*/
  uint8_t ScanMode; /*Scanning mode,   0  Single-ended input  8 channelÃ¯Â¿Â½Ã¯Â¿Â½ 1 Differential input  4 channel*/
}ADS1256_VAR_T;

static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
  0xF0,   /*reset the default values  */
  0xE0,
  0xD0,
  0xC0,
  0xB0,
  0xA1,
  0x92,
  0x82,
  0x72,
  0x63,
  0x53,
  0x43,
  0x33,
  0x20,
  0x13,
  0x03
};



class ADS1256 {
  public:

  ADS1256(int cs);
  ADS1256(int cs, int drdy); 
  ADS1256(int cs, int drdy, ADS1256_DRATE_E _drate);

  void setNumChannels(int n); 
  void startCollecting();
  void stopCollecting(void);
  bool collectData(void);
  
  bool configADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

  float* get_data(int channel);
  
  uint8_t ADS1256_ReadReg(uint8_t _RegID);
  void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
  
  int bufferSize(void);

  void reset(void);
  
  ~ADS1256();

  private: 
  int numChannels = 8;

  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch0;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch1;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch2;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch3;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch4;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch5;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch6;
  FastDualBuffer<float,ADS1256_BUFFERSIZE> data_ch7;

  FastDualBuffer<float,ADS1256_BUFFERSIZE> * bufferptrs[8];
  
  uint8_t ADS1256_ReadChipID(void);
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


